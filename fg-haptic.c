// Haptic
#include <stdlib.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_haptic.h>
#include <SDL2/SDL_net.h>

#include <stdio.h>              /* printf */
#include <string.h>             /* strstr */
#include <ctype.h>              /* isdigit */

#include <stdbool.h>		/* bool */
#include <math.h>

#include <signal.h>

#include <time.h>

/* From fgfsclient */
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdarg.h>


#define DFLTHOST        "localhost"
#define DFLTPORT        5401
#define MAXMSG          512
#define fgfsclose       SDLNet_TCP_Close

#define NAMELEN		30

// Currently supported effects:
// 0) Constant force = pilot G and control surface loading
// 1) Rumble = stick shaker
#define TIMEOUT		1   // 1 sec
#define READ_TIMEOUT	5   // 5 secs
#define CONN_TIMEOUT	30    // 30 seconds
#define AXES		3 	// Maximum axes supported

#define CONST_X		0
#define CONST_Y		1
#define CONST_Z		2
#define STICK_SHAKER	3
#define FRICTION	4
#define DAMPER		5

#define EFFECTS		9

const char axes[AXES] = {'x', 'y', 'z'};

//void init_sockaddr(struct sockaddr_in *name, const char *hostname, unsigned port);
TCPsocket fgfsconnect(const char *hostname, const int port, bool server);
int fgfswrite(TCPsocket sock, char *msg, ...);
const char *fgfsread(TCPsocket sock, int wait);
void fgfsflush(TCPsocket sock);

// Socket used to communicate with flightgear
TCPsocket telnet_sock, server_sock, client_sock;
SDLNet_SocketSet socketset;

// Effect struct definitions, used to store parameters
typedef struct __effectParams {
    float pilot[AXES];
    float stick[AXES];
    int shaker_trigger;
    float rumble_period;	// Ground rumble period, 0=disable

    float x;  // Forces
    float y;
    float z;
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
    float rumble_gain;

    // TODO: Possibility to invert axes
    signed char pilot_axes[AXES];  // Axes mapping, -1 = not used
    signed char stick_axes[AXES];

    unsigned int last_rumble;

    float lowpass;  // Low pass filter tau, in ms

} hapticDevice;

static hapticDevice *devices = NULL;
bool reconf_request = false;
bool quit = false;

effectParams new_params;


#define CLAMP(x, l, h) ((x)>(h)?(h):((x)<(l)?(l):(x)))


/*
 * prototypes
 */
void abort_execution(int signal);
void HapticPrintSupported(SDL_Haptic * haptic);

void init_haptic(void)
{
    /* Initialize the force feedbackness */
    SDL_Init(SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC);

    // Initialize network
    SDLNet_Init();

    num_devices = SDL_NumHaptics();
    printf("%d Haptic devices detected.\n", num_devices);

    devices = (hapticDevice*)malloc(num_devices * sizeof(hapticDevice));
    if(!devices) {
        printf("Fatal error: Could not allocate memory for devices!\n");
        abort_execution(-1);
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

          HapticPrintSupported(devices[i].device);
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
          if(devices[i].axes > AXES) devices[i].axes = AXES;
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
          devices[i].rumble_gain = 0.2;
          devices[i].lowpass = 300.0;

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

          fgfswrite(telnet_sock, "set /haptic/device[%d]/low-pass-filter %.6f", i, devices[i].lowpass);


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
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/supported 1", i);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/pilot/supported 1", i);

              fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/period 0.0", i);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/gain %f", i, devices[i].rumble_gain);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/supported 1", i);
          }

          if(devices[i].supported & SDL_HAPTIC_SINE)
          {
              // Sine effect -> rumble is stick shaker
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/direction %f", i, devices[i].shaker_dir);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/period %f", i, devices[i].shaker_period);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/gain %f", i, devices[i].shaker_gain);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/trigger 0", i);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/supported 1", i);
          }

          if(devices[i].supported & SDL_HAPTIC_GAIN) {
              fgfswrite(telnet_sock, "set /haptic/device[%d]/gain %f", i, devices[i].gain);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/gain-supported 1", i);
          }
          if(devices[i].supported & SDL_HAPTIC_AUTOCENTER) {
              fgfswrite(telnet_sock, "set /haptic/device[%d]/autocenter %f", i, devices[i].autocenter);
              fgfswrite(telnet_sock, "set /haptic/device[%d]/autocenter-supported 1", i);
          }
    }
}

void read_devices(void)
{
    int idata;
    float fdata;
    int read = 0;
    const char *p;

    fgfsflush(telnet_sock);

    printf("Reading device setup from FG\n");

    for(int i=0; i < num_devices; i++)
    {
        // Constant device settings
        fgfswrite(telnet_sock, "get /haptic/device[%d]/low-pass-filter", i);
        p = fgfsread(telnet_sock, READ_TIMEOUT);
        if(p) {
            read = sscanf(p, "%f", &fdata);
            if(read == 1) devices[i].lowpass = fdata;
        }

        if(devices[i].supported & SDL_HAPTIC_GAIN) {
            fgfswrite(telnet_sock, "get /haptic/device[%d]/gain", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].gain = fdata;
            }
        }

        if(devices[i].supported & SDL_HAPTIC_AUTOCENTER) {
            fgfswrite(telnet_sock, "get /haptic/device[%d]/autocenter", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].autocenter = fdata;
            }
        }

        // Constant force -> pilot G forces and aileron loading
        // Currently support 3 axis only
        if(devices[i].supported & SDL_HAPTIC_CONSTANT) {
            for(int x=0; x<devices[i].axes && x<AXES; x++) {
                fgfswrite(telnet_sock, "get /haptic/device[%d]/pilot/%c", i, axes[x]);
                p = fgfsread(telnet_sock, READ_TIMEOUT);
                if(p) {
                    read = sscanf(p, "%d", &idata);
                    if(read == 1) devices[i].pilot_axes[x] = idata;
                }

                fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/%c", i, axes[x]);
                p = fgfsread(telnet_sock, READ_TIMEOUT);
                if(p) {
                    read = sscanf(p, "%d", &idata);
                    if(read == 1) devices[i].stick_axes[x] = idata;
                }
            }
            fgfswrite(telnet_sock, "get /haptic/device[%d]/pilot/gain", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].pilot_gain = fdata;
            }
            fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/gain", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].stick_gain = fdata;
            }

            fgfswrite(telnet_sock, "get /haptic/device[%d]/ground-rumble/gain", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].rumble_gain = fdata;
            }
        }


        if(devices[i].supported & SDL_HAPTIC_SINE) {
            fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/direction", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].shaker_dir = fdata;
            }
            fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/period", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].shaker_period = fdata;
            }
            fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/gain", i);
            p = fgfsread(telnet_sock, READ_TIMEOUT);
            if(p) {
                read = sscanf(p, "%f", &fdata);
                if(read == 1) devices[i].shaker_gain = fdata;
            }
        }
    }

    fgfswrite(telnet_sock, "set /haptic/reconfigure 0");
    printf("Waiting for the command to go through...\n");
    do {
        fgfswrite(telnet_sock, "get /haptic/reconfigure");
        p = fgfsread(telnet_sock, READ_TIMEOUT);
        if(p) read = sscanf(p, "%d", &idata);
        else read = 0;
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

        printf("Creating effects for device %d\n", i+1);

        // Set autocenter and gain
        if(devices[i].supported & SDL_HAPTIC_AUTOCENTER)
            SDL_HapticSetAutocenter(devices[i].device, devices[i].autocenter * 100);

        if(devices[i].supported & SDL_HAPTIC_GAIN)
            SDL_HapticSetGain(devices[i].device, devices[i].gain * 100);

        // Stick shaker
        if (devices[i].supported & SDL_HAPTIC_SINE && devices[i].shaker_gain > 0.001)
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
                abort_execution(-1);
            }
        }

        // X axis
        if (devices[i].supported & SDL_HAPTIC_CONSTANT && devices[i].axes > 0)
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
                abort_execution(-1);
            }
        }

        // Y axis
        if (devices[i].supported & SDL_HAPTIC_CONSTANT && devices[i].axes > 1)
        {
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
                abort_execution(-1);
            }
        }

        // Z axis
        if (devices[i].supported & SDL_HAPTIC_CONSTANT && devices[i].axes > 2)
        {
            devices[i].effect[CONST_Z].type = SDL_HAPTIC_CONSTANT;
            devices[i].effect[CONST_Z].constant.direction.type = SDL_HAPTIC_CARTESIAN;
            devices[i].effect[CONST_Z].constant.direction.dir[0] = 0;
            devices[i].effect[CONST_Z].constant.direction.dir[1] = 0;
            devices[i].effect[CONST_Z].constant.direction.dir[2] = 0x1000;
            devices[i].effect[CONST_Z].constant.length = 60000;  // By default constant fore is always applied
            devices[i].effect[CONST_Z].constant.level = 0x1000;

            devices[i].effectId[CONST_Z] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_Z]);
            if(devices[i].effectId[CONST_Z] < 0) {
                printf("UPLOADING CONST_Y EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution(-1);
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
    const char *p;

    p = fgfsread(client_sock, TIMEOUT);
    if(!p) return;  // Null pointer, read failed

    memset(&new_params, 0, sizeof(effectParams));

    // Divide the buffer into chunks
    read = sscanf(p, "%d|%f|%f|%f|%f|%f|%f|%d|%f", &reconf,
                   &new_params.pilot[0], &new_params.pilot[1], &new_params.pilot[2],
		   &new_params.stick[0], &new_params.stick[1], &new_params.stick[2],
		   &new_params.shaker_trigger, &new_params.rumble_period);

    if(read != 9) {
        printf("Error reading generic I/O!\n");
        return;
    }

    // printf("%s, %d\n", p, reconf);

    // Do it the easy way...
    // memcpy(&devices[0].params, &new_params, sizeof(effectParams));

    if(reconf & 1) reconf_request = true;
}


/**
 * @brief The entry point of this force feedback demo.
 * @param[in] argc Number of arguments.
 * @param[in] argv Array of argc arguments.
 */
int
main(int argc, char **argv)
{
    int i = 0;
    char *name = NULL;
    struct sigaction signal_handler;
    effectParams *oldParams = NULL;
    unsigned int runtime = 0;
    unsigned int dt = 0;

    // Handlers for ctrl+c etc quitting methods
    signal_handler.sa_handler = abort_execution;
    sigemptyset(&signal_handler.sa_mask);
    signal_handler.sa_flags = 0;
    sigaction(SIGINT, &signal_handler, NULL);
    sigaction(SIGQUIT, &signal_handler, NULL);


    printf("fg-haptic version 0.5\n");
    printf("Force feedback support for Flight Gear\n");
    printf("Copyright 2011, 2014 Lauri Peltonen, released under GPLv2 or later\n\n");

    if (argc > 1) {
        name = argv[1];
        if ((strcmp(name, "--help") == 0) || (strcmp(name, "-h") == 0)) {
            printf("USAGE: %s\n"
                   "No parameters yet. Telnet port for FlightGear is %d and generic\n"
                   "is %d. See Readme for details.\n",
                   argv[0], DFLTPORT, DFLTPORT+1);
            return 0;
        }
    }

    // Initialize SDL haptics
    init_haptic();

    // Create & upload force feedback effects
    create_effects();

    socketset = SDLNet_AllocSocketSet(2);
    if(!socketset) {
        printf("Unable to create socket set: %s\n", SDLNet_GetError());
        abort_execution(-1);
    }

    // Wait for a connection from flightgear generic io
    printf("\n\nWaiting for flightgear generic IO at port %d, please run Flight Gear now!\n", DFLTPORT+1);
    server_sock = fgfsconnect(DFLTHOST, DFLTPORT+1, true);
    if(!server_sock) {
        printf("Failed to connect!\n");
        abort_execution(-1);
    }

    printf("Got connection, sending haptic details through telnet at port %d\n", DFLTPORT);

    // Connect to flightgear using telnet
    telnet_sock = fgfsconnect(DFLTHOST, DFLTPORT, false);
    if (!telnet_sock) {
        printf("Could not connect to flightgear with telnet!\n");
        abort_execution(-1);
    }

    // Add sockets to a socket set for polling/selecting
    SDLNet_TCP_AddSocket(socketset, client_sock);
    SDLNet_TCP_AddSocket(socketset, telnet_sock);

    // Switch to data mode
    fgfswrite(telnet_sock, "data");

    // send the devices to flightgear
    send_devices();


    // allocate memory for old param values
    oldParams = (effectParams *)malloc(num_devices * sizeof(effectParams));
    if(!oldParams) {
        printf("Fatal error: Could not allocate memory!\n");
        abort_execution(-1);
    }



    // Main loop

    while(!quit)  // Loop as long as the connection is alive
    {
        dt = runtime;
        runtime = SDL_GetTicks();  // Run time in ms
        dt = runtime - dt;

        // Back up old parameters
        for(int i=0; i < num_devices; i++)
            memcpy((void *)&oldParams[i], (void *)&devices[i].params, sizeof(effectParams));

        memset((void *)&devices[i].params, 0, sizeof(effectParams));

        // Read new parameters
        read_fg();

        // If parameters have changed, apply them
        for(int i=0; i < num_devices; i++)
        {
            if(!devices[i].device || !devices[i].open) continue;  // Break if device is not opened correctly

            // Constant forces (stick forces, pilot G forces
	    if((devices[i].supported & SDL_HAPTIC_CONSTANT))
            {
                // Stick forces with axis mapping
                if(devices[i].stick_axes[0] >= 0)
                    devices[i].params.x = new_params.stick[devices[i].stick_axes[0]] * devices[i].stick_gain;
                if(devices[i].stick_axes[1] >= 0)
                    devices[i].params.y = new_params.stick[devices[i].stick_axes[1]] * devices[i].stick_gain;
                if(devices[i].stick_axes[2] >= 0)
                    devices[i].params.z = new_params.stick[devices[i].stick_axes[2]] * devices[i].stick_gain;

                // Pilot forces
                if(devices[i].stick_axes[0] >= 0)
                    devices[i].params.x += new_params.stick[devices[i].stick_axes[0]] * devices[i].stick_gain;
                if(devices[i].pilot_axes[1] >= 0)
                    devices[i].params.y += new_params.pilot[devices[i].pilot_axes[1]] * devices[i].pilot_gain;
                if(devices[i].pilot_axes[2] >= 0)
                    devices[i].params.z += new_params.pilot[devices[i].pilot_axes[2]] * devices[i].pilot_gain;


                // Add ground rumble
                if(devices[i].params.rumble_period > 0.00001) {
                    if((runtime - devices[i].last_rumble) > devices[i].params.rumble_period) {
                        devices[i].params.y += devices[i].rumble_gain;
                        devices[i].last_rumble = runtime;
                    }
                }

                devices[i].params.x *= 32760.0;
                devices[i].params.y *= 32760.0;
                devices[i].params.z *= 32760.0;

                // Low pass filter
                float g1 = ((float)dt / (devices[i].lowpass + dt));
                float g2 = (devices[i].lowpass / (devices[i].lowpass + dt));
                devices[i].params.x = devices[i].params.x * g1 + oldParams[i].x * g2;
                devices[i].params.y = devices[i].params.y * g1 + oldParams[i].y * g2;
                devices[i].params.z = devices[i].params.z * g1 + oldParams[i].z * g2;

                if(devices[i].axes > 0 && devices[i].effectId[CONST_X] != -1) {
                    devices[i].effect[CONST_X].constant.level = (signed short)CLAMP(devices[i].params.x, -32760.0, 32760.0);
                    reload_effect(&devices[i], &devices[i].effect[CONST_X], &devices[i].effectId[CONST_X], true);
                }
                if(devices[i].axes > 1 && devices[i].effectId[CONST_Y] != -1) {
                    devices[i].effect[CONST_Y].constant.level = (signed short)CLAMP(devices[i].params.y, -32760.0, 32760.0);;
                    reload_effect(&devices[i], &devices[i].effect[CONST_Y], &devices[i].effectId[CONST_Y], true);
                }
                if(devices[i].axes > 2 && devices[i].effectId[CONST_Z] != -1) {
                    devices[i].effect[CONST_Z].constant.level = (signed short)CLAMP(devices[i].params.z, -32760.0, 32760.0);;
                    reload_effect(&devices[i], &devices[i].effect[CONST_Z], &devices[i].effectId[CONST_Z], true);
                }

		// printf("dt: %d  X: %.6f  Y: %.6f\n", (unsigned int)dt, devices[i].params.x, devices[i].params.y);
            }

            // Stick shaker trigger
            if((devices[i].supported & SDL_HAPTIC_SINE) && devices[i].effectId[STICK_SHAKER] != -1)
            {
                if(devices[i].params.shaker_trigger && !oldParams[i].shaker_trigger)
                    reload_effect(&devices[i], &devices[i].effect[STICK_SHAKER], &devices[i].effectId[STICK_SHAKER], true);
                else if(!devices[i].params.shaker_trigger && oldParams[i].shaker_trigger)
                    SDL_HapticStopEffect(devices[i].device, devices[i].effectId[STICK_SHAKER]);
            }
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

    SDLNet_FreeSocketSet(socketset);

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
void abort_execution(int signal)
{
    printf("\nAborting program execution.\n");

    // Close flightgear telnet connection
    fgfswrite(telnet_sock, "quit");
    fgfsclose(telnet_sock);

    // Adn generic
    fgfsclose(client_sock);
    fgfsclose(server_sock);

    SDLNet_FreeSocketSet(socketset);

    // Close haptic devices
    for(int i=0; i < num_devices; i++)
      if(devices[i].open && devices[i].device) SDL_HapticClose(devices[i].device);

    if(devices) free(devices);
    devices = NULL;

    SDLNet_Quit();

    SDL_Quit();

    exit(1);
}


/*
 * Displays information about the haptic device.
 */
void HapticPrintSupported(SDL_Haptic *haptic)
{
    unsigned int supported;

    supported = SDL_HapticQuery(haptic);
    printf("   Device has %d axis\n", SDL_HapticNumAxes(haptic));
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



int fgfswrite(TCPsocket sock, char *msg, ...)
{
        va_list va;
        ssize_t len;
        char buf[MAXMSG];

        if(!sock) return 0;

        va_start(va, msg);
        vsnprintf(buf, MAXMSG - 2, msg, va);
        va_end(va);
        //printf("SEND: \t<%s>\n", buf);
        strcat(buf, "\r\n");

        len = SDLNet_TCP_Send(sock, buf, strlen(buf));
        if (len < 0) {
                printf("Error in fgfswrite: %s\n", SDLNet_GetError());
                exit(EXIT_FAILURE);
        }
        return len;
}

const char *fgfsread(TCPsocket sock, int timeout)
{
        static char buf[MAXMSG];
        char *p = &buf[0];
        size_t len = 0;
        int ready = 0;
        time_t start;

        memset(buf, 0, MAXMSG);

        start = clock();
        do {
            ready = SDLNet_CheckSockets(socketset, timeout*1000);
            if(!ready) {
                //printf("Timeout!\n");
                return NULL;
            }

            // TODO: Loop until socket is ready or timeout?
            if(SDLNet_SocketReady(sock)) {
                ready = 1;
            }
        } while(!ready && clock() < (start+timeout*CLOCKS_PER_SEC));

        if(!ready) {
            //printf("Error in fgfsread: Socket was not ready (timeout)!\n");
            return NULL;
        }

        ready = 0;
        do {
            if(SDLNet_TCP_Recv(sock, p, 1) <= 0) {
                // printf("Error in fgfsread: Recv returned zero!\n");
                quit = true;
                return NULL;
            }
            len++;
            if(len == MAXMSG - 1) {
                printf("Warning in fgfsread: Buffer size exceeded!\n");
                ready = 1;
            } else if(*p == '\n') {
                ready = 1;
            }
            p++;
        } while(!ready);


        for (p = &buf[len - 1]; p >= buf; p--)
                if (*p != '\r' && *p != '\n')
                        break;
        *++p = '\0';

        // if(strlen(buf)) printf("RECV: %s\n", buf);

        return strlen(buf) ? buf : NULL;
}



void fgfsflush(TCPsocket sock)
{
    const char *p;
    while ((p = fgfsread(sock, 0)) != NULL) {
        //printf("IGNORE: \t<%s>\n", p);
    }
}

TCPsocket fgfsconnect(const char *hostname, const int port, bool server)
{
    IPaddress serv_addr, cli_addr;
    TCPsocket _sock, _clientsock;
    time_t start;

    if(!server)  // Act as a client -> connect to address
    {
        if(SDLNet_ResolveHost(&cli_addr, hostname, port) == -1)
        {
            printf("Error in fgfsconnect, resolve host: %s\n", SDLNet_GetError());
            return NULL;
        }

        _sock = SDLNet_TCP_Open(&cli_addr);
        if(!_sock) {
            printf("Error in fgfsconnect, connect: %s\n", SDLNet_GetError());
            return NULL;
        }

        return _sock;

    } else { // Act as a server, wait for connections
        if(SDLNet_ResolveHost(&serv_addr, NULL, port) == -1)
        {
            printf("Error in fgfsconnect, server resolve host: %s\n", SDLNet_GetError());
            return NULL;
        }

        _sock = SDLNet_TCP_Open(&serv_addr);
        if(!_sock) {
            printf("Error in fgfsconnect, server connect: %s\n", SDLNet_GetError());
            return NULL;
        }

        // Wait for connection until timeout
        start = clock();
        do {
            // TODO: Add a small sleep here
            _clientsock = SDLNet_TCP_Accept(_sock);
        } while(!_clientsock && clock() < (start+CONN_TIMEOUT*CLOCKS_PER_SEC));

        if(!_clientsock) {
            printf("Error in fgfsconnect: Connection timeout\n");
            return NULL;
        }

        client_sock = _clientsock;

        return _sock;
    }
}

