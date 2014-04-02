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

/* Flightgear properties used:

YASim at least:
/accelerations/pilot/x-accel-fps_sec
/accelerations/pilot/y-accel-fps_sec
/accelerations/pilot/z-accel-fps_sec
*/

#define DFLTHOST        "localhost"
#define DFLTPORT        5401
#define MAXMSG          512
#define fgfsclose       close

#define NAMELEN		30

// Currently supported effects:
// 0) Constant force = pilot G and control surface loading
// 1) Rumble = stick shaker
#define TIMEOUT		400000
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
int sock, server_sock, client_sock;


// Device supports (workarounds...)
typedef struct __deviceHacks {
    bool liveUpdate;  // Support updating while effect is running (BEST CASE)?
    bool update;      // Support update at all... bad case if not
} deviceHacks;


// Effect struct definitions, used to store parameters
typedef struct __effectParams {
    float autocenter;
    float gain;

    float pilot[AXES];
    float pilot_gain;

    float surface[AXES];
    float surface_gain;

    float shaker_dir;
    float shaker_gain;
    float shaker_period;
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

    deviceHacks hacks;

    SDL_HapticEffect effect[EFFECTS];
    int effectId[EFFECTS];

    effectParams params;
} hapticDevice;

static hapticDevice *devices = NULL;


#define CLAMP(x, l, h) ((x)>(h)?(h):((x)<(l)?(l):(x)))


/*
 * prototypes
 */
static void abort_execution(void);
static void HapticPrintSupported(SDL_Haptic * haptic);

void check_hacks(hapticDevice *device)
{
    // Testing with sine rumble, since it is best supported
    if (device->supported & SDL_HAPTIC_SINE)
    {
        device->effect[0].type = SDL_HAPTIC_SINE;
        device->effect[0].periodic.direction.type = SDL_HAPTIC_POLAR;
        device->effect[0].periodic.direction.dir[0] = 0;
        device->effect[0].periodic.direction.dir[1] = 0;
        device->effect[0].periodic.direction.dir[2] = 0;

        device->effect[0].periodic.length = 1000;
        device->effect[0].periodic.period = 1000;
        device->effect[0].periodic.magnitude = 0x0001; // Should not feel it when testing...

        device->effect[0].periodic.attack_length = 500; // 1 sec fade in
        device->effect[0].periodic.fade_length = 500; // 1 sec fade out

        device->effectId[0] = SDL_HapticNewEffect(device->device, &device->effect[0]);
        if(device->effectId[0] < 0) {
            printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
            abort_execution();
        }

        // Try to update when effect is not running
        if(SDL_HapticUpdateEffect(device->device, device->effectId[0], &device->effect[0]) < 0) device->hacks.update = false;
        else device->hacks.update=true;

        // Now run it and try live update
        SDL_HapticRunEffect(device->device, device->effectId[0], 1);
        if(SDL_HapticUpdateEffect(device->device, device->effectId[0], &device->effect[0]) < 0) device->hacks.liveUpdate = false;
        else device->hacks.liveUpdate = true;
        SDL_HapticDestroyEffect(device->device, device->effectId[0]);

        printf("Device supports update: %d, liveUpdate: %d\n", device->hacks.update, device->hacks.liveUpdate);

        // Empty the effect struct
        memset(&device->effect[0], 0, sizeof(SDL_HapticEffect));
        device->effectId[0] = 0;
    }
    else if (device->supported & SDL_HAPTIC_CONSTANT)  // Let's try with constant if sine is not available...
    {
        device->effect[0].type = SDL_HAPTIC_CONSTANT;

        device->effect[0].constant.direction.type = SDL_HAPTIC_CARTESIAN;
        device->effect[0].constant.direction.dir[0] = 0;
        device->effect[0].constant.direction.dir[1] = 0;
        device->effect[0].constant.direction.dir[2] = 0;

        device->effect[0].constant.length = 1000;  // By default constant fore is always applied
        device->effect[0].constant.level = 0x0001;

        device->effectId[0] = SDL_HapticNewEffect(device->device, &device->effect[0]);
        if(device->effectId[0] < 0) {
            printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
            abort_execution();
        }

        // Try to update when effect is not running
        if(SDL_HapticUpdateEffect(device->device, device->effectId[0], &device->effect[0]) < 0) device->hacks.update = false;
        else device->hacks.update=true;

        // Now run it and try live update
        SDL_HapticRunEffect(device->device, device->effectId[0], 1);
        if(SDL_HapticUpdateEffect(device->device, device->effectId[0], &device->effect[0]) < 0) device->hacks.liveUpdate = false;
        else device->hacks.liveUpdate = true;
        SDL_HapticDestroyEffect(device->device, device->effectId[0]);

        printf("Device supports update: %d, liveUpdate: %d\n", device->hacks.update, device->hacks.liveUpdate);

        // Empty the effect struct
        memset(&device->effect[0], 0, sizeof(SDL_HapticEffect));
        device->effectId[0] = 0;
    }
}


void init_haptic(void)
{
    /* Initialize the force feedbackness */
    SDL_Init(/*SDL_INIT_VIDEO |*/ SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC);

    num_devices = SDL_NumHaptics();
    printf("%d Haptic devices detected.\n", num_devices);

    devices = (hapticDevice*)malloc(num_devices * sizeof(hapticDevice));
    if(!devices) {
        printf("Fatal error: Could not allocate memory for devices!\n");
        abort();
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

          // Test device support
          //check_hacks(&devices[i]);

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
    for(int i=0;i < num_devices; i++)
    {
          // Write devices to flightgear
          fgfswrite(sock, "set /haptic/device[%d]/number %d", i, devices[i].num);
          fgfswrite(sock, "set /haptic/device[%d]/name %s", i, devices[i].name);
          fgfswrite(sock, "set /haptic/device[%d]/supported %d", i, devices[i].supported);
          fgfswrite(sock, "set /haptic/device[%d]/axes %d", i, devices[i].axes);
          fgfswrite(sock, "set /haptic/device[%d]/num-effects %d", i, devices[i].numEffects);
          fgfswrite(sock, "set /haptic/device[%d]/num-effects-playing %d", i, devices[i].numEffectsPlaying);

          // Write supported effects
          if(devices[i].supported & SDL_HAPTIC_CONSTANT)
          {
              // Constant force -> pilot G forces and aileron loading
              // Currently support 3 axis only
              for(int x=0;x<devices[i].axes && x<AXES; x++) {
                  fgfswrite(sock, "set /haptic/device[%d]/pilot/%c %d", i, axes[x], 0);
                  fgfswrite(sock, "set /haptic/device[%d]/stick-force/%c %d", i, axes[x], 0);
              }
              fgfswrite(sock, "set /haptic/device[%d]/pilot/gain %f", i, 0.5);
              fgfswrite(sock, "set /haptic/device[%d]/stick-force/gain %f", i, 0.5);

              // fgfswrite(sock, "set /haptic/device[%d]/pilot/supported 1", i);
              // fgfswrite(sock, "set /haptic/device[%d]/stick-force/supported 1", i);
          }

          if(devices[i].supported & SDL_HAPTIC_SINE)
          {
              // Sine effect -> rumble is stick shaker
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/direction %d", i, 90);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/period %d", i, 50);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/gain %d", i, 1);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/trigger %d", i, 0);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/supported 1", i);
          }

          if(devices[i].supported & SDL_HAPTIC_GAIN) {
              fgfswrite(sock, "set /haptic/device[%d]/gain %d", i, 1);
              // fgfswrite(sock, "set /haptic/device[%d]/gain-supported 1", i);
          }
          if(devices[i].supported & SDL_HAPTIC_AUTOCENTER) {
              fgfswrite(sock, "set /haptic/device[%d]/autocenter %d", i, 0);
              // fgfswrite(sock, "set /haptic/device[%d]/autocenter-supported 1", i);
          }
    }
}

void create_effects(void)
{
    for(int i=0; i < num_devices; i++)
    {
        memset(&devices[i].effect[0],0 , sizeof(SDL_HapticEffect)*EFFECTS);

        if (devices[i].supported & SDL_HAPTIC_SINE)
        {
            devices[i].effect[STICK_SHAKER].type = SDL_HAPTIC_SINE;
            devices[i].effect[STICK_SHAKER].periodic.direction.type = SDL_HAPTIC_POLAR;
            devices[i].effect[STICK_SHAKER].periodic.direction.dir[0] = 0;
            devices[i].effect[STICK_SHAKER].periodic.direction.dir[1] = 0;
            devices[i].effect[STICK_SHAKER].periodic.direction.dir[2] = 0;
            devices[i].effect[STICK_SHAKER].periodic.length = 5000;  // Default 10 seconds?
            devices[i].effect[STICK_SHAKER].periodic.period = 100;    // 100 ms period = 10 Hz?
            devices[i].effect[STICK_SHAKER].periodic.magnitude = 0x4000;
            devices[i].effect[STICK_SHAKER].periodic.attack_length = 1000; // 1 sec fade in
            devices[i].effect[STICK_SHAKER].periodic.fade_length = 1000; // 1 sec fade out

            devices[i].effectId[STICK_SHAKER] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[STICK_SHAKER]);
            if(devices[i].effectId[STICK_SHAKER] < 0) {
                printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
        }

        // First effect is constant force
        printf("Creating effects for device %d\n", i);
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

  // TODO: Obsolete code below
#if 0
    //printf("Updating device %d, effect %d\n", device->num, *effectId);

    // If we can update effects, either live or not, do it!
    if(device->hacks.update)
    {
        if(!device->hacks.liveUpdate) if(SDL_HapticStopEffect(device->device, *effectId) < 0) printf("Error: %s\n", SDL_GetError());
        if(SDL_HapticUpdateEffect(device->device, *effectId, effect) < 0) printf("Update error: %s\n", SDL_GetError());
        if(run) if(SDL_HapticRunEffect(device->device, *effectId, 1) < 0) printf("Run error: %s\n", SDL_GetError());

    // If we can update, and can run one more effect simultaneously
    }
    else if(run && device->numEffects > EFFECTS && device->numEffectsPlaying > EFFECTS)
    {
        int oldId = *effectId;
        if(((*effectId) = SDL_HapticNewEffect(device->device, effect)) < 0) printf("Error: %s\n", SDL_GetError());
        if(SDL_HapticRunEffect(device->device, *effectId, 1) < 0) printf("Error: %s\n", SDL_GetError());
        SDL_HapticDestroyEffect(device->device, oldId);

    // Otherwise must first delete, then create new (and launch it)
    }
    else
    {
        SDL_HapticDestroyEffect(device->device, *effectId);
        if(((*effectId) = SDL_HapticNewEffect(device->device, effect)) < 0) printf("Error: %s\n", SDL_GetError());
        if(run) if(SDL_HapticRunEffect(device->device, *effectId, 1) < 0) printf("Error: %s\n", SDL_GetError());
    }
#endif
}


void read_fg(void)
{
    int reconf, read;
    effectParams params;
    const char *p;

    p = fgfsread(client_sock, TIMEOUT);
    if(!p) return;  // Null pointer, read failed

    memset(&params, 0, sizeof(effectParams));

    //printf("%s\n", p);

    // Divide the buffer into chunks
    read = sscanf(p, "%d|%f|%f|%f|%f|%f|%f|%d", &reconf,
                   &params.pilot[0], &params.pilot[1], &params.pilot[2],
		   &params.surface[0], &params.surface[1], &params.surface[2],
		   &params.shaker_trigger);

    if(read != 8) {
        printf("Error reading generic I/O!\n");
        return;
    }

    // Do it the easy way...
    memcpy(&devices[0].params, &params, sizeof(effectParams));
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
    // Open generic I/O connection to flightgear
    printf("\n\nWaiting for flightgear generic IO at port %d, please run Flight Gear now!\n", DFLTPORT+1);
    server_sock = fgfsconnect(DFLTHOST, DFLTPORT+1, true);

    printf("Got connection, sending haptic details through telnet at port %d\n", DFLTPORT);

    // Connect to flightgear
    sock = fgfsconnect(DFLTHOST, DFLTPORT, false);
    if (sock < 0) {
        printf("Could not connect to flightgear!\n");
        return EXIT_FAILURE;
    }

    // Switch to data mode
    fgfswrite(sock, "data");

    // send the devices to flightgear
    send_devices();

    // Close flightgear telnet connection
    fgfswrite(sock, "quit");
    fgfsclose(sock);

    // allocate memory for old param values
    oldParams = (effectParams *)malloc(num_devices * sizeof(effectParams));
    if(!oldParams) {
        printf("Fatal error: Could not allocate memory!\n");
        abort();
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
	    if(devices[i].supported & SDL_HAPTIC_CONSTANT)
            {
//                float x = devices[i].params.pilot_gain*devices[i].params.pilot[0] + devices[i].params.surface_gain*devices[i].params.surface[0];
//                float y = devices[i].params.pilot_gain*devices[i].params.pilot[1] + devices[i].params.surface_gain*devices[i].params.surface[1];
//                float z = devices[i].params.pilot_gain*devices[i].params.pilot[2] + devices[i].params.surface_gain*devices[i].params.surface[2];

                float x = devices[i].params.surface[0];
                float y = devices[i].params.surface[1];
                float z = devices[i].params.surface[2];

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
            if(devices[i].supported & SDL_HAPTIC_SINE && devices[i].params.shaker_trigger && !oldParams[i].shaker_trigger)
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
    }


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
    fgfswrite(sock, "quit");
    fgfsclose(sock);

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
#ifdef USE_GENERIC
#else
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
#endif
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
          // printf("Timeout!\n");
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
                printf("IGNORE: \t<%s>\n", p);
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

