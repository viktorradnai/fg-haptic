// Haptic
#include <stdlib.h>
#ifdef SDL2
#include <SDL2/SDL.h>
#include <SDL2/SDL_haptic.h>
#else
#include <SDL/SDL.h>
#include <SDL/SDL_haptic.h>
#endif

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

    int effectId[EFFECTS];
    SDL_HapticEffect effect[EFFECTS];

    effectParams params;
} hapticDevice;

static hapticDevice *devices = NULL;

int trigger = 0;



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
          SDL_HapticSetGain(devices[i].device, 100);

          // Test device support
          //check_hacks(&devices[i]);
          HapticPrintSupported(devices[i].device);

        } else {
            printf("Unable to open haptic devices %d: %s\n", i, SDL_GetError());
            devices[i].open = false;
        }
    }

    /* We only want force feedback errors. */
    SDL_ClearError();
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
            devices[i].effect[STICK_SHAKER].periodic.length = 3000;  // Default 10 seconds?
            devices[i].effect[STICK_SHAKER].periodic.period = 100;    // 100 ms period = 10 Hz?
            devices[i].effect[STICK_SHAKER].periodic.magnitude = 0x4000;
            devices[i].effect[STICK_SHAKER].periodic.attack_length = 000; // 1 sec fade in
            devices[i].effect[STICK_SHAKER].periodic.fade_length = 000; // 1 sec fade out

            devices[i].effectId[STICK_SHAKER] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[STICK_SHAKER]);
            if(devices[i].effectId[STICK_SHAKER] < 0) {
                printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }

            for(int j=0;j<3;j++) printf("Id %d\n", devices[i].effectId[j]);
        }

        // First effect is constant force
        printf("Creating effects for device %d\n", i);
        if (devices[i].supported & SDL_HAPTIC_CONSTANT)
        {
            devices[i].effect[CONST_X].type = SDL_HAPTIC_SPRING;
            devices[i].effect[CONST_X].condition.direction.type = SDL_HAPTIC_CARTESIAN;
            devices[i].effect[CONST_X].condition.direction.dir[0] = 1;
            devices[i].effect[CONST_X].condition.direction.dir[1] = 1;
            devices[i].effect[CONST_X].condition.direction.dir[2] = 0;
            devices[i].effect[CONST_X].condition.right_sat[0] = 0x7FFF;
            devices[i].effect[CONST_X].condition.right_sat[1] = 0x7FFF;
//            devices[i].effect[CONST_X].condition.right_sat[2] = 0x7FFF;
            devices[i].effect[CONST_X].condition.left_sat[0] = 0x7FFF;
            devices[i].effect[CONST_X].condition.left_sat[1] = 0x7FFF;
//            devices[i].effect[CONST_X].condition.left_sat[2] = 0x7FFF;
            devices[i].effect[CONST_X].condition.right_coeff[0] = 0x7FFF;
            devices[i].effect[CONST_X].condition.right_coeff[1] = 0x7FFF;
//            devices[i].effect[CONST_X].condition.right_coeff[2] = 0x7FFF;
            devices[i].effect[CONST_X].condition.left_coeff[0] = 0x7FFF;
            devices[i].effect[CONST_X].condition.left_coeff[1] = 0x7FFF;
//            devices[i].effect[CONST_X].condition.left_coeff[2] = 0x7FFF;
            devices[i].effect[CONST_X].condition.length = 10000;
            devices[i].effect[CONST_X].condition.deadband[0] = 0;
            devices[i].effect[CONST_X].condition.deadband[1] = 0;
            devices[i].effect[CONST_X].condition.center[0] = 0;
            devices[i].effect[CONST_X].condition.center[1] = 0;

            devices[i].effectId[CONST_X] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_X]);
            if(devices[i].effectId[CONST_X] < 0) {
                printf("UPLOADING CONST_X EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
            for(int j=0;j<3;j++) printf("Id %d\n", devices[i].effectId[j]);
/*
            devices[i].effect[CONST_Y].type = SDL_HAPTIC_CONSTANT;
            devices[i].effect[CONST_Y].constant.direction.type = SDL_HAPTIC_CARTESIAN;
            devices[i].effect[CONST_Y].constant.direction.dir[0] = 0;
            devices[i].effect[CONST_Y].constant.direction.dir[1] = 0x1000;
            devices[i].effect[CONST_Y].constant.direction.dir[2] = 0;
            devices[i].effect[CONST_Y].constant.length = 60000;  // By default constant fore is always applied
            devices[i].effect[CONST_Y].constant.level = 0x1000;

            devices[i].effectId[CONST_Y] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_Y]);
            if(devices[i].effectId[CONST_Y] < 0) {
                printf("UPLOADING CONST_Y EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
            for(int j=0;j<3;j++) printf("Id %d\n", devices[i].effectId[j]);
*/
        }
    }
}


void reload_effect(hapticDevice *device, SDL_HapticEffect *effect, int *effectId, bool run)
{
    if(!device->device || !device->open) return;

    printf("Updating device %d, effect %d\n", device->num, *effectId);

        //if(!device->hacks.liveUpdate) if(SDL_HapticStopEffect(device->device, *effectId) < 0) printf("Error: %s\n", SDL_GetError());
        if(SDL_HapticUpdateEffect(device->device, *effectId, effect) < 0) printf("Update error: %s\n", SDL_GetError());
        if(run) if(SDL_HapticRunEffect(device->device, *effectId, 1) < 0) printf("Run error: %s\n", SDL_GetError());

#if 0
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
    int read;
    effectParams params;

    memset(&params, 0, sizeof(effectParams));

    read = getchar();
    if(read == 'q') abort_execution();
    if(read == '1') trigger = 1;

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
    float t = 0.0;

    effectParams *oldParams = NULL;

    name = NULL;

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
            name = NULL;
        }
    }

    // Initialize SDL haptics
    init_haptic();

    // Create & upload force feedback effects
    create_effects();

    // Wait for a connection from flightgear generic io

    // allocate memory for old param values
    oldParams = (effectParams *)malloc(num_devices * sizeof(effectParams));
    if(!oldParams) {
        printf("Fatal error: Could not allocate memory!\n");
        abort();
    }




    // Main loop

    // Wait until connection gives an error
    while(1)  // Loop as long as the connection is alive
    {
        // Back up old parameters
        for(int i=0; i < num_devices; i++)
            memcpy((void *)&oldParams[i], (void *)&devices[i].params, sizeof(effectParams));

        // Read new parameters
        read_fg();

        // If parameters have changed, apply them
        for(int i=0; i < num_devices; i++)
        {
            if(!devices[i].device || !devices[i].open) continue;  // Break if device is not opened correctly

            // Check for parameter changes
            // Pilot G force changed?
	    if(1)
            {

		if(t>1.0) t-=1.0;
                float x = t * 0x7F00;
                float y = cos(t) * 0x7F00;

                devices[i].effect[CONST_X].condition.right_coeff[0] = x;
                devices[i].effect[CONST_X].condition.right_coeff[1] = x;
                devices[i].effect[CONST_X].condition.right_coeff[2] = 0x1000;
                devices[i].effect[CONST_X].condition.left_coeff[0] = x;
                devices[i].effect[CONST_X].condition.left_coeff[1] = x;
                devices[i].effect[CONST_X].condition.left_coeff[2] = 0x1000;

                //devices[i].effect[CONST_X].constant.level = (signed short)(x);
                //devices[i].effect[CONST_Y].constant.level = (signed short)(y);

		printf("X: %.6f  Y: %.6f\n", x, y);
		//printf("X: %6d  Y: %6d\n", devices[i].effect[CONST_X].constant.level, devices[i].effect[CONST_Y].constant.level);

                // If updating is supported, do it
                reload_effect(&devices[i], &devices[i].effect[CONST_X], &devices[i].effectId[CONST_X], true);
                //reload_effect(&devices[i], &devices[i].effect[CONST_Y], &devices[i].effectId[CONST_Y], true);
            }

            if(trigger) {
                reload_effect(&devices[i], &devices[i].effect[STICK_SHAKER], &devices[i].effectId[STICK_SHAKER], true);
                trigger = 0;
            }

#if 0
            if(devices[i].supported & SDL_HAPTIC_SINE && devices[i].params.shaker_trigger && !oldParams[i].shaker_trigger)
            {
                // printf("Rumble triggered! %u %d\n", (unsigned int)devices[i].device, devices[i].effectId[1]);
                reload_effect(&devices[i], &devices[i].effect[1], &devices[i].effectId[1], true);

                // The following line seems to cause crash, so using the reload_effect way!
                // if(SDL_HapticRunEffect(devices[i].device, devices[i].effectId[1], 1) < 0) printf("Error3: %s\n", SDL_GetError());
            }

            if(devices[i].supported & SDL_HAPTIC_AUTOCENTER && devices[i].params.autocenter != oldParams[i].autocenter) SDL_HapticSetAutocenter(devices[i].device, devices[i].params.autocenter * 100);
            if(devices[i].supported & SDL_HAPTIC_GAIN && devices[i].params.gain != oldParams[i].gain) SDL_HapticSetGain(devices[i].device, devices[i].params.gain * 100);
#endif
        }

        t += 0.05;
    }


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

