// Haptic
#include <stdlib.h>
#include <SDL.h>
#include <SDL_haptic.h>

#include <stdio.h>              /* printf */
#include <string.h>             /* strstr */
#include <ctype.h>              /* isdigit */

#include <stdbool.h>		/* bool */
#include <math.h>
#include <sys/poll.h>

/* From fgfsclient */
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdarg.h>


// #define USE_GENERIC 1

#define DFLTHOST        "localhost"
#define DFLTPORT        5403
#define MAXMSG          512
#define fgfsclose       close

#define NAMELEN		30

// Currently supported effects:
// 0) Constant force = pilot G and control surface loading
// 1) Rumble = stick shaker
#define TIMEOUT		1e6   // 1 sec
#define READ_TIMEOUT	5e6   // 5 secs
#define EFFECTS		9
#define AXES		3 	// Maximum axes supported

#define CONST_X		0
#define CONST_Y		1
#define STICK_SHAKER	2

const char axes[AXES] = {'x', 'y', 'z'};

void init_sockaddr(struct sockaddr_in *name, const char *hostname, unsigned port);
int fgfsconnect(const char *hostname, const int port, bool server);
int fgfswrite(int sock, char *msg, ...);
const char *fgfsread(int sock, int wait);
void fgfsflush(int sock);

// Socket used to communicate with flightgear
int telnet_sock, server_sock, client_sock;

// Effect struct definitions, used to store parameters
typedef struct __effectParams {
    float pilot[AXES];
    float stick[AXES];
    int shaker_trigger;
} effectParams;

int num_devices;

typedef struct __hapticdevice {
    SDL_Haptic *device;
    char name[NAMELEN+1];          // Name
    unsigned int num;       // Num of this device
    unsigned int supported; // Capabilities
    unsigned int axes;		// Count of axes
    unsigned int numEffects, numEffectsPlaying;
    bool open;

    SDL_HapticEffect effect[EFFECTS];
    int effectId[EFFECTS];

    effectParams params;

    // Configuration
    float autocenter;
    float gain;

    unsigned short shaker_dir;
    unsigned short shaker_period;

    float pilot_gain;
    float stick_gain;
    float shaker_gain;

    // TODO: Possibility to invert axes
    signed char pilot_axes[AXES];  // Axes mapping, -1 = not used
    signed char stick_axes[AXES];

} hapticDevice;

static hapticDevice *devices = NULL;
bool reconf_request = false;

#define CLAMP(x, l, h) ((x)>(h)?(h):((x)<(l)?(l):(x)))


/*
 * prototypes
 */
static void abort_execution(void);
static void HapticPrintSupported(SDL_Haptic * haptic);

void init_haptic(void)
{
    /* Initialize the force feedbackness */
    SDL_Init(/*SDL_INIT_VIDEO |*/ SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC);

    num_devices = SDL_NumHaptics();
    printf("%d Haptic devices detected.\n", num_devices);

    devices = (hapticDevice*)malloc(num_devices * sizeof(hapticDevice));
    if(!devices) {
        printf("Fatal error: Could not allocate memory for devices!\n");
        abort_execution();
    }

    // Zero
    memset(devices, 0, num_devices * sizeof(hapticDevice));

    // Send all devices' data to flightgear
    for(int i=0;i < num_devices; i++)
    {
        devices[i].num = i + 1;  // Add one, so we get around flightgear reading empty properties as 0
        devices[i].device = SDL_HapticOpen(i);

        if(devices[i].device) {
          devices[i].open = true;

          // Copy devices name with ascii
          const char *p = SDL_HapticName(i);
          strncpy(devices[i].name, p, NAMELEN);

          // Add device number after name, if there is multiples with same name
          for(int a=0; a < i; a++) {
               if(strcmp(devices[i].name, devices[a].name) == 0) {
                   size_t len = strlen(devices[i].name);
                   if(len < NAMELEN - 2) { // Enough space to add number after name
                       devices[i].name[len] = ' ';
                       devices[i].name[len+1] = '1' + i;
                   } else {
                       devices[i].name[NAMELEN-2] = ' ';
                       devices[i].name[NAMELEN-1] = '1' + i;
                   }
               }
          }

          printf("Device %d name is %s\n", devices[i].num, devices[i].name);

          // Capabilities
          devices[i].supported = SDL_HapticQuery(devices[i].device);
          devices[i].axes = SDL_HapticNumAxes(devices[i].device);
          devices[i].numEffects = SDL_HapticNumEffects(devices[i].device);
          devices[i].numEffectsPlaying = SDL_HapticNumEffectsPlaying(devices[i].device);

          // Default effect parameters
          for(int a = 0; a < devices[i].axes && a < AXES; a++) {
              devices[i].pilot_axes[a] = a;
              devices[i].stick_axes[a] = a;
          }

          devices[i].autocenter = 0.0;
          devices[i].gain = 1.0;
          devices[i].pilot_gain = 0.1;
          devices[i].stick_gain = 1.0;
          devices[i].shaker_gain = 1.0;
          devices[i].shaker_period = 100.0;

        } else {
            printf("Unable to open haptic devices %d: %s\n", i, SDL_GetError());
            devices[i].open = false;
        }
    }

    /* We only want force feedback errors. */
    SDL_ClearError();
}

void send_devices(void)
{
    // Init general properties
    fgfswrite(telnet_sock, "set /haptic/reconfigure 0");

    // Init devices
    for(int i=0;i < num_devices; i++)
    {
          // Write devices to flightgear
          fgfswrite(telnet_sock, "set /haptic/device[%d]/number %d", i, devices[i].num);
          fgfswrite(telnet_sock, "set /haptic/device[%d]/name %s", i, devices[i].name);
          fgfswrite(telnet_sock, "set /haptic/device[%d]/supported %d", i, devices[i].supported);
          fgfswrite(telnet_sock, "set /haptic/device[%d]/axes %d", i, devices[i].axes);
          fgfswrite(telnet_sock, "set /haptic/device[%d]/num-effects %d", i, devices[i].numEffects);
          fgfswrite(telnet_sock, "set /haptic/device[%d]/num-effects-playing %d", i, devices[i].numEffectsPlaying);

          // Write supported effects
          if(devices[i].supported & SDL_HAPTIC_CONSTANT)
          {
              // Constant force -> pilot G forces and aileron loading
              // Currently support 3 axis only
              for(int x=0;x<devices[i].axes && x<AXES; x++) {
                  fgfswrite(telnet_sock, "set /haptic/device[%d]/pilot/%c %d", i, axes[x], devices[i].pilot_axes[x]);
                  fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/%c %d", i, axes[x], devices[i].stick_axes[x]);
              }
              fgfswrite(telnet_sock, "set /haptic/device[%d]/pilot/gain %f", i, devices[i].pilot_gain);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/gain %f", i, devices[i].stick_gain);
          }

          if(devices[i].supported & SDL_HAPTIC_SINE)
          {
              // Sine effect -> rumble is stick shaker
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/direction %f", i, devices[i].shaker_dir);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/period %f", i, devices[i].shaker_period);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/gain %f", i, devices[i].shaker_gain);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/trigger 0", i);
              // fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/supported 1", i);
          }

          if(devices[i].supported & SDL_HAPTIC_GAIN) {
              fgfswrite(telnet_sock, "set /haptic/device[%d]/gain %f", i, devices[i].gain);
          }
          if(devices[i].supported & SDL_HAPTIC_AUTOCENTER) {
              fgfswrite(telnet_sock, "set /haptic/device[%d]/autocenter %f", i, devices[i].autocenter);
          }
    }
}

void read_devices(void)
{
    int idata;
    float fdata;
    int read;

    fgfsflush(telnet_sock);

    printf("Reading device setup from FG\n");

    for(int i=0;i < num_devices; i++)
    {
        // Constant device settings
        fgfswrite(telnet_sock, "get /haptic/device[%d]/gain", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].gain = fdata;
        fgfswrite(telnet_sock, "get /haptic/device[%d]/autocenter", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].autocenter = fdata;

        // Constant force -> pilot G forces and aileron loading
        // Currently support 3 axis only
        for(int x=0; x<devices[i].axes && x<AXES; x++) {
            fgfswrite(telnet_sock, "get /haptic/device[%d]/pilot/%c", i, axes[x]);
            read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%d", &idata);
            if(read == 1) devices[i].pilot_axes[x] = idata;

            fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/%c", i, axes[x]);
            read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%d", &idata);
            if(read == 1) devices[i].stick_axes[x] = idata;
        }
        fgfswrite(telnet_sock, "get /haptic/device[%d]/pilot/gain", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].pilot_gain = fdata;
        fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/gain", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].stick_gain = fdata;

        fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/direction", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].shaker_dir = fdata;
        fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/period", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].shaker_period = fdata;
        fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/gain", i);
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%f", &fdata);
        if(read == 1) devices[i].shaker_gain = fdata;
    }

    fgfswrite(telnet_sock, "set /haptic/reconfigure 0");
    printf("Waiting for the command to go through...\n");
    do {
        fgfswrite(telnet_sock, "get /haptic/reconfigure");
        read = sscanf(fgfsread(telnet_sock, READ_TIMEOUT), "%d", &idata);
    } while(read == 1 && idata == 1);
    printf("Done\n");
    fgfsflush(client_sock);  // Get rid of FF data that was received during reinitialization
    return;
}


void create_effects(void)
{
    for(int i=0; i < num_devices; i++)
    {
        // Delete existing effects
        for(int x=0; x < SDL_HapticNumEffects(devices[i].device); x++) {
            SDL_HapticDestroyEffect(devices[i].device, x);
            devices[i].effectId[x] = -1;
        }

        memset(&devices[i].effect[0],0 , sizeof(SDL_HapticEffect)*EFFECTS);

        printf("Creating effects for device %d\n", i);

        // Stick shaker
        if (devices[i].supported & SDL_HAPTIC_SINE && devices[i].shaker_gain > 0.0)
        {
            devices[i].effect[STICK_SHAKER].type = SDL_HAPTIC_SINE;
            devices[i].effect[STICK_SHAKER].periodic.direction.type = SDL_HAPTIC_POLAR;
            devices[i].effect[STICK_SHAKER].periodic.direction.dir[0] = devices[i].shaker_dir;
            devices[i].effect[STICK_SHAKER].periodic.direction.dir[1] = 0;
            devices[i].effect[STICK_SHAKER].periodic.direction.dir[2] = 0;
            devices[i].effect[STICK_SHAKER].periodic.length = 5000;  // Default 5 seconds?
            devices[i].effect[STICK_SHAKER].periodic.period = devices[i].shaker_period;
            devices[i].effect[STICK_SHAKER].periodic.magnitude = 0x4000;
            devices[i].effect[STICK_SHAKER].periodic.attack_length = 300; // 0.3 sec fade in
            devices[i].effect[STICK_SHAKER].periodic.fade_length = 300; // 0.3 sec fade out

            devices[i].effectId[STICK_SHAKER] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[STICK_SHAKER]);
            if(devices[i].effectId[STICK_SHAKER] < 0) {
                printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
        }

        if (devices[i].supported & SDL_HAPTIC_CONSTANT)
        {
            devices[i].effect[CONST_X].type = SDL_HAPTIC_CONSTANT;
            devices[i].effect[CONST_X].constant.direction.type = SDL_HAPTIC_CARTESIAN;
            devices[i].effect[CONST_X].constant.direction.dir[0] = 0x1000;
            devices[i].effect[CONST_X].constant.direction.dir[1] = 0;
            devices[i].effect[CONST_X].constant.direction.dir[2] = 0;
            devices[i].effect[CONST_X].constant.length = 60000;  // By default constant fore is always applied
            devices[i].effect[CONST_X].constant.level = 0x1000;

            devices[i].effectId[CONST_X] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_X]);
            if(devices[i].effectId[CONST_X] < 0) {
                printf("UPLOADING CONST_X EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }

            devices[i].effect[CONST_Y].type = SDL_HAPTIC_CONSTANT;
            devices[i].effect[CONST_Y].constant.direction.type = SDL_HAPTIC_CARTESIAN;
            devices[i].effect[CONST_Y].constant.direction.dir[0] = 0;
            devices[i].effect[CONST_Y].constant.direction.dir[1] = -0x1000;
            devices[i].effect[CONST_Y].constant.direction.dir[2] = 0;
            devices[i].effect[CONST_Y].constant.length = 60000;  // By default constant fore is always applied
            devices[i].effect[CONST_Y].constant.level = 0x1000;

            devices[i].effectId[CONST_Y] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_Y]);
            if(devices[i].effectId[CONST_Y] < 0) {
                printf("UPLOADING CONST_Y EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
        }
    }
}


void reload_effect(hapticDevice *device, SDL_HapticEffect *effect, int *effectId, bool run)
{
    if(!device->device || !device->open) return;

    if(SDL_HapticUpdateEffect(device->device, *effectId, effect) < 0) printf("Update error: %s\n", SDL_GetError());
    if(run) if(SDL_HapticRunEffect(device->device, *effectId, 1) < 0) printf("Run error: %s\n", SDL_GetError());
}


void read_fg(void)
{
    int reconf, read;
    effectParams params;
    const char *p;

    p = fgfsread(client_sock, TIMEOUT);
    if(!p) return;  // Null pointer, read failed

    memset(&params, 0, sizeof(effectParams));

    // Divide the buffer into chunks
    read = sscanf(p, "%d|%f|%f|%f|%f|%f|%f|%d", &reconf,
                   &params.pilot[0], &params.pilot[1], &params.pilot[2],
		   &params.stick[0], &params.stick[1], &params.stick[2],
		   &params.shaker_trigger);

    if(read != 8) {
        printf("Error reading generic I/O!\n");
        return;
    }

    // printf("%s, %d\n", p, reconf);

    // Do it the easy way...
    memcpy(&devices[0].params, &params, sizeof(effectParams));

    if(reconf == 1) reconf_request=true;
}


/**
 * @brief The entry point of this force feedback demo.
 * @param[in] argc Number of arguments.
 * @param[in] argv Array of argc arguments.
 */
int
main(int argc, char **argv)
{
    int i;
    char *name;
    int index;
    int nefx;
    unsigned int supported;
    const char *p;
    struct pollfd clientpoll;

    effectParams *oldParams = NULL;

    name = NULL;
    index = -1;

    printf("fg-haptic version 0.1\n");
    printf("Force feedback support for Flight Gear\n");
    printf("Copyright 2011 Lauri Peltonen, released under GPLv2 or later\n\n");

    if (argc > 1) {
        name = argv[1];
        if ((strcmp(name, "--help") == 0) || (strcmp(name, "-h") == 0)) {
            printf("USAGE: %s [device]\n"
                   "If device is a two-digit number it'll use it as an index, otherwise\n"
                   "it'll use it as if it were part of the device's name.\n",
                   argv[0]);
            return 0;
        }

        i = strlen(name);
        if ((i < 3) && isdigit(name[0]) && ((i == 1) || isdigit(name[1]))) {
            index = atoi(name);
            name = NULL;
        }
    }

    // Initialize SDL haptics
    init_haptic();

    // Create & upload force feedback effects
    create_effects();

    // Wait for a connection from flightgear generic io
    printf("\n\nWaiting for flightgear generic IO at port %d, please run Flight Gear now!\n", DFLTPORT+1);
    server_sock = fgfsconnect(DFLTHOST, DFLTPORT+1, true);
    if(server_sock < 0) {
        printf("Failed to connect!\n");
        abort_execution();
    }

    printf("Got connection, sending haptic details through telnet at port %d\n", DFLTPORT);

    // Connect to flightgear using telnet
    telnet_sock = fgfsconnect(DFLTHOST, DFLTPORT, false);
    if (telnet_sock < 0) {
        printf("Could not connect to flightgear with telnet!\n");
        abort_execution();
    }

    // Switch to data mode
    fgfswrite(telnet_sock, "data");

    // send the devices to flightgear
    send_devices();


    // allocate memory for old param values
    oldParams = (effectParams *)malloc(num_devices * sizeof(effectParams));
    if(!oldParams) {
        printf("Fatal error: Could not allocate memory!\n");
        abort_execution();
    }



    // Main loop

    // Wait until connection gives an error
    clientpoll.fd = client_sock;
    clientpoll.events = POLLIN | POLLPRI; // Dunno about these..
    clientpoll.revents = 0;
    while(1)  // Loop as long as the connection is alive
    {

        poll(&clientpoll, 1, 0);

        // Back up old parameters
        for(int i=0; i < num_devices; i++)
            memcpy((void *)&oldParams[i], (void *)&devices[i].params, sizeof(effectParams));

        // Read new parameters
        read_fg();

        // If parameters have changed, apply them
        for(int i=0; i < num_devices; i++)
        {
            if(!devices[i].device || !devices[i].open) continue;  // Break if device is not opened correctly

            // Constant forces (stick forces, pilot G forces
            // TODO: Pilot G forces
	    if((devices[i].supported & SDL_HAPTIC_CONSTANT) && devices[i].effectId[CONST_X] != -1 &&
               devices[i].effectId[CONST_Y] != -1)
            {
//                float x = devices[i].params.pilot_gain*devices[i].params.pilot[0] + devices[i].params.surface_gain*devices[i].params.surface[0];
//                float y = devices[i].params.pilot_gain*devices[i].params.pilot[1] + devices[i].params.surface_gain*devices[i].params.surface[1];
//                float z = devices[i].params.pilot_gain*devices[i].params.pilot[2] + devices[i].params.surface_gain*devices[i].params.surface[2];

                float x = 0.0;
                float y = 0.0;
                float z = 0.0;

                // Stick forces with axis mapping
                if(devices[i].stick_axes[0] >= 0)
                    x = devices[i].params.stick[devices[i].stick_axes[0]] * devices[i].stick_gain;
                if(devices[i].stick_axes[1] >= 0)
                    y = devices[i].params.stick[devices[i].stick_axes[1]] * devices[i].stick_gain;
                if(devices[i].stick_axes[2] >= 0)
                    z = devices[i].params.stick[devices[i].stick_axes[2]] * devices[i].stick_gain;

                // Pilot forces
                if(devices[i].pilot_axes[0] >= 0)
                    x += devices[i].params.pilot[devices[i].pilot_axes[0]] * devices[i].pilot_gain;
                if(devices[i].pilot_axes[1] >= 0)
                    y += devices[i].params.pilot[devices[i].pilot_axes[1]] * devices[i].pilot_gain;
                if(devices[i].pilot_axes[2] >= 0)
                    z += devices[i].params.pilot[devices[i].pilot_axes[2]] * devices[i].pilot_gain;

                x = CLAMP(x, -1.0, 1.0) * 32760.0;
                y = CLAMP(y, -1.0, 1.0) * 32760.0;
                z = CLAMP(z, -1.0, 1.0) * 32760.0;

                // Normalize direction vector so we don't saturate it
                devices[i].effect[CONST_X].constant.level = (signed short)(x);
                devices[i].effect[CONST_Y].constant.level = (signed short)(y);

		printf("X: %.6f  Y: %.6f\n", x, y);
		//printf("X: %6d  Y: %6d\n", devices[i].effect[CONST_X].constant.level, devices[i].effect[CONST_Y].constant.level);

                // If updating is supported, do it
                reload_effect(&devices[i], &devices[i].effect[CONST_X], &devices[i].effectId[CONST_X], true);
                reload_effect(&devices[i], &devices[i].effect[CONST_Y], &devices[i].effectId[CONST_Y], true);
            }

            // Stick shaker trigger
            if(devices[i].supported & SDL_HAPTIC_SINE && devices[i].params.shaker_trigger &&
               !oldParams[i].shaker_trigger && devices[i].effectId[STICK_SHAKER] != -1)
            {
                //printf("Rumble triggered! %u %d\n", (unsigned int)devices[i].device, devices[i].effectId[1]);
                reload_effect(&devices[i], &devices[i].effect[STICK_SHAKER], &devices[i].effectId[STICK_SHAKER], true);
            }
#if 0
            if(devices[i].supported & SDL_HAPTIC_SINE && (devices[i].params.shaker_dir != oldParams[i].shaker_dir ||
               devices[i].params.shaker_gain != oldParams[i].shaker_gain || devices[i].params.shaker_period != oldParams[i].shaker_period))
            {
                // printf("Update stick shaker\n");
                devices[i].effect[1].periodic.direction.dir[0] = devices[i].params.shaker_dir*100.0;

                devices[i].effect[1].periodic.length = 5000;  // Default 5 seconds?
                devices[i].effect[1].periodic.period = devices[i].params.shaker_period;
                devices[i].effect[1].periodic.magnitude = (unsigned short)(devices[i].params.shaker_gain * 0x3FFF) & 0x7FFF;

                reload_effect(&devices[i], &devices[i].effect[1], &devices[i].effectId[1], false);
            }

            if(devices[i].supported & SDL_HAPTIC_AUTOCENTER && devices[i].params.autocenter != oldParams[i].autocenter) SDL_HapticSetAutocenter(devices[i].device, devices[i].params.autocenter * 100);
            if(devices[i].supported & SDL_HAPTIC_GAIN && devices[i].params.gain != oldParams[i].gain) SDL_HapticSetGain(devices[i].device, devices[i].params.gain * 100);
#endif
        }

        if(reconf_request)
        {
            reconf_request = false;
            read_devices();
            create_effects();
        }
    }


    // Close flightgear telnet connection
    fgfswrite(telnet_sock, "quit");
    fgfsclose(telnet_sock);

    // Close generic connection
    fgfsclose(client_sock);
    fgfsclose(server_sock);

    if(oldParams) free(oldParams);
    oldParams = NULL;

    // Close haptic devices
    for(int i=0; i < num_devices; i++)
      if(/*devices[i].open &&*/ devices[i].device) SDL_HapticClose(devices[i].device);

    if(devices) free(devices);
    devices = NULL;

    SDL_Quit();

    return 0;
}


/*
 * Cleans up a bit.
 */
static void
abort_execution(void)
{
    printf("\nAborting program execution.\n");

    // Close flightgear telnet connection
    fgfswrite(telnet_sock, "quit");
    fgfsclose(telnet_sock);

    // Adn generic
    fgfsclose(client_sock);
    fgfsclose(server_sock);

    // Close haptic devices
    for(int i=0; i < num_devices; i++)
      if(devices[i].open && devices[i].device) SDL_HapticClose(devices[i].device);

    if(devices) free(devices);
    devices = NULL;

    SDL_Quit();

    exit(1);
}


/*
 * Displays information about the haptic device.
 */
static void
HapticPrintSupported(SDL_Haptic * haptic)
{
    unsigned int supported;

    supported = SDL_HapticQuery(haptic);
    printf("   Supported effects [%d effects, %d playing]:\n",
           SDL_HapticNumEffects(haptic), SDL_HapticNumEffectsPlaying(haptic));
    if (supported & SDL_HAPTIC_CONSTANT)
        printf("      constant\n");
    if (supported & SDL_HAPTIC_SINE)
        printf("      sine\n");
/*    if (supported & SDL_HAPTIC_SQUARE)
        printf("      square\n");*/
    if (supported & SDL_HAPTIC_TRIANGLE)
        printf("      triangle\n");
    if (supported & SDL_HAPTIC_SAWTOOTHUP)
        printf("      sawtoothup\n");
    if (supported & SDL_HAPTIC_SAWTOOTHDOWN)
        printf("      sawtoothdown\n");
    if (supported & SDL_HAPTIC_RAMP)
        printf("      ramp\n");
    if (supported & SDL_HAPTIC_FRICTION)
        printf("      friction\n");
    if (supported & SDL_HAPTIC_SPRING)
        printf("      spring\n");
    if (supported & SDL_HAPTIC_DAMPER)
        printf("      damper\n");
    if (supported & SDL_HAPTIC_INERTIA)
        printf("      intertia\n");
    if (supported & SDL_HAPTIC_CUSTOM)
        printf("      custom\n");
    printf("   Supported capabilities:\n");
    if (supported & SDL_HAPTIC_GAIN)
        printf("      gain\n");
    if (supported & SDL_HAPTIC_AUTOCENTER)
        printf("      autocenter\n");
    if (supported & SDL_HAPTIC_STATUS)
        printf("      status\n");
}



int fgfswrite(int sock, char *msg, ...)
{
        va_list va;
        ssize_t len;
        char buf[MAXMSG];

        va_start(va, msg);
        vsnprintf(buf, MAXMSG - 2, msg, va);
        va_end(va);
        //printf("SEND: \t<%s>\n", buf);
        strcat(buf, "\015\012");

        len = write(sock, buf, strlen(buf));
        if (len < 0) {
                perror("fgfswrite");
                exit(EXIT_FAILURE);
        }
        return len;
}

const char *fgfsread(int sock, int timeout)
{
        static char buf[MAXMSG];
        char *p;
        fd_set ready;
        struct timeval tv;
        ssize_t len;

        memset(buf, 0, MAXMSG);

        FD_ZERO(&ready);
        FD_SET(sock, &ready);
        tv.tv_sec = 0;
        tv.tv_usec = timeout;
        if (!select(32, &ready, 0, 0, &tv)) {
            printf("Timeout!\n");
            return NULL;
        }

        len = read(sock, buf, MAXMSG - 1);
        if (len < 0) {
                perror("fgfsread");
                exit(EXIT_FAILURE);
        }
        if (len == 0)
                return NULL;

        // if(strlen(buf)) printf("%s\n\n", buf);

        for (p = &buf[len - 1]; p >= buf; p--)
                if (*p != '\015' && *p != '\012')
                        break;
        *++p = '\0';

        return strlen(buf) ? buf : NULL;
}



void fgfsflush(int sock)
{
        const char *p;
        while ((p = fgfsread(sock, 0)) != NULL) {
                //printf("IGNORE: \t<%s>\n", p);
        }
}

int fgfsconnect(const char *hostname, const int port, bool server)
{
        struct sockaddr_in serv_addr, cli_addr;
        socklen_t cli_size;
        struct hostent *hostinfo;
	int _sock, _clientsock;

        _sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (_sock < 0) {
                perror("fgfsconnect/socket");
                return -1;
        }

        hostinfo = gethostbyname(hostname);
        if (hostinfo == NULL) {
                fprintf(stderr, "fgfsconnect: unknown host: \"%s\"\n", hostname);
                close(_sock);
                return -2;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        serv_addr.sin_addr = *(struct in_addr *)hostinfo->h_addr_list[0];

	if(!server)  // Act as a client -> connect to address
	{
	        if (connect(_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        	        perror("fgfsconnect/connect");
                	close(_sock);
	                return -3;
		}
        } else { // Act as a server, wait for connections
		if(bind(_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        	        perror("fgfsconnect/bind");
                	close(_sock);
	                return -3;
		}
		listen(_sock, 1); // Wait for maximum of 1 conenction
                cli_size = sizeof(cli_addr);
		_clientsock = accept(_sock, (struct sockaddr *)&cli_addr, &cli_size);
		if(_clientsock < 0) {
        	        perror("fgfsconnect/accept");
                	close(_sock);
	                return -3;
		}
		client_sock = _clientsock;
	}
        return _sock;
}

