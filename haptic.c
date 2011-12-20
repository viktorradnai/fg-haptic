// Haptic
#include <stdlib.h>
#include <SDL.h>
#include <SDL_haptic.h>

#include <stdio.h>              /* printf */
#include <string.h>             /* strstr */
#include <ctype.h>              /* isdigit */

#include <stdbool.h>		/* bool */
#include <math.h>

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
#define MAXMSG          256
#define fgfsclose       close

#define NAMELEN		30

// Currently supported effects:
// 0) Constant force = pilot G and control surface loading
// 1) Rumble = stick shaker
#define TIMEOUT		400000
#define EFFECTS		2
#define AXES		3 	// Maximum axes supported

const char axes[AXES] = {'x', 'y', 'z'};

void init_sockaddr(struct sockaddr_in *name, const char *hostname, unsigned port);
int fgfsconnect(const char *hostname, const int port);
int fgfswrite(int sock, char *msg, ...);
const char *fgfsread(int sock, int wait);
void fgfsflush(int sock);

// Socket used to communicate with flightgear
int sock;


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
    char shaker_trigger;
} effectParams;

int num_devices;

typedef struct __hapticdevice {
    SDL_Haptic *device;
    char name[NAMELEN];          // Name
    unsigned int num;       // Num of this device
    unsigned int supported; // Capabilities
    unsigned int axes;		// Count of axes
    unsigned int numEffects, numEffectsPlaying;
    bool open;

    SDL_HapticEffect effect[EFFECTS];
    int effectId[EFFECTS];

    effectParams params;
} hapticDevice;

static hapticDevice *devices = NULL;
static SDL_Haptic *haptic = NULL;  // For compatibility, TODO: remove!




/*
 * prototypes
 */
static void abort_execution(void);
static void HapticPrintSupported(SDL_Haptic * haptic);


void init_haptic(void)
{
    /* Initialize the force feedbackness */
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC);

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
        devices[i].num = i;
        devices[i].open = true;
        devices[i].device = SDL_HapticOpen(i);

        if(devices[i].device) {
          // Copy devices name with ascii
          const char *p = SDL_HapticName(i);
          strncpy(devices[i].name, p, NAMELEN-1);
          printf("Device names is %s\n", devices[i].name);


          // Capabilities
          devices[i].supported = SDL_HapticQuery(devices[i].device);
          devices[i].axes = SDL_HapticNumAxes(devices[i].device);
          devices[i].numEffects = SDL_HapticNumEffects(devices[i].device);
          devices[i].numEffectsPlaying = SDL_HapticNumEffectsPlaying(devices[i].device);

          // Write devices to flightgear
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
                  fgfswrite(sock, "set /haptic/device[%d]/surface-force/%c %d", i, axes[x], 0);
              }
              fgfswrite(sock, "set /haptic/device[%d]/pilot/gain %f", i, 0.5);
              fgfswrite(sock, "set /haptic/device[%d]/surface-force/gain %f", i, 0.5);
          }

          if(devices[i].supported & SDL_HAPTIC_SINE)
          {
              // Sine effect -> rumble is stick shaker
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/direction %d", i, 90);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/period %d", i, 50);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/gain %d", i, 1);
              fgfswrite(sock, "set /haptic/device[%d]/stick-shaker/trigger %d", i, 0);
          }

          if(devices[i].supported & SDL_HAPTIC_GAIN) fgfswrite(sock, "set /haptic/device[%d]/gain %d", i, 1);
          if(devices[i].supported & SDL_HAPTIC_AUTOCENTER) fgfswrite(sock, "set /haptic/device[%d]/autocenter %d", i, 0);

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
        // First effect is constant force
        printf("Create effects for device %d\n", i);
        if (devices[i].supported & SDL_HAPTIC_CONSTANT)
        {
            devices[i].effect[0].type = SDL_HAPTIC_CONSTANT;

            devices[i].effect[0].constant.direction.type = SDL_HAPTIC_CARTESIAN;
            devices[i].effect[0].constant.direction.dir[0] = 1;
            devices[i].effect[0].constant.direction.dir[1] = 0;
            devices[i].effect[0].constant.direction.dir[2] = 0;

            devices[i].effect[0].constant.length = 60000;  // By default constant fore is always applied
            devices[i].effect[0].constant.level = 0x2000;

            devices[i].effectId[0] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[0]);
            if(devices[i].effectId[0] < 0) {
                printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
            // if(SDL_HapticRunEffect(devices[i].device, devices[i].effectId[0], 1) < 0) printf("Error3: %s\n", SDL_GetError());
            printf("Constant done, id %d\n", devices[i].effectId[0]);
        }

        if (devices[i].supported & SDL_HAPTIC_SINE)
        {
            devices[i].effect[1].type = SDL_HAPTIC_SINE;

            devices[i].effect[1].periodic.direction.type = SDL_HAPTIC_POLAR;
            devices[i].effect[1].periodic.direction.dir[0] = 0;
            devices[i].effect[1].periodic.direction.dir[1] = 0;
            devices[i].effect[1].periodic.direction.dir[2] = 0;

            devices[i].effect[1].periodic.length = 5000;  // Default 10 seconds?
            devices[i].effect[1].periodic.period = 50;    // 100 ms period = 10 Hz?
            devices[i].effect[1].periodic.magnitude = 0x4000;

            devices[i].effect[1].periodic.attack_length = 1000; // 1 sec fade in
            devices[i].effect[1].periodic.fade_length = 1000; // 1 sec fade out

            devices[i].effectId[1] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[1]);
            if(devices[i].effectId[1] < 0) {
                printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
                abort_execution();
            }
            //if(SDL_HapticRunEffect(devices[i].device, devices[i].effectId[1], 1) < 0) printf("Error3: %s\n", SDL_GetError());
            printf("Rumble done, id %d\n", devices[i].effectId[1]);
        }
    }
}

void read_fg(void)
{
    const char *p;

    for(int i=0; i < num_devices; i++)
    {
        if(devices[i].supported & SDL_HAPTIC_CONSTANT)
        {
            for(int x=0; x<devices[i].axes && x<AXES; x++)
            {
                fgfswrite(sock, "get /haptic/device[%d]/pilot/%c", i, axes[x]);
                p = fgfsread(sock, TIMEOUT);
                if (p != NULL) devices[i].params.pilot[x] = atof(p);

                fgfswrite(sock, "get /haptic/device[%d]/surface-force/%c", i, axes[x]);
                p = fgfsread(sock, TIMEOUT);
                if (p != NULL) devices[i].params.surface[x] = atof(p);
            }

            fgfswrite(sock, "get /haptic/device[%d]/pilot/gain", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.pilot_gain = atof(p);

            fgfswrite(sock, "get /haptic/device[%d]/surface-force/gain", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.surface_gain = atof(p);
        }

        if(devices[i].supported & SDL_HAPTIC_SINE)
        {
            fgfswrite(sock, "get /haptic/device[%d]/stick-shaker/direction", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.shaker_dir = atof(p);

            fgfswrite(sock, "get /haptic/device[%d]/stick-shaker/gain", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.shaker_gain = atof(p);

            fgfswrite(sock, "get /haptic/device[%d]/stick-shaker/period", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.shaker_period = atof(p);
        }

        if(devices[i].supported & SDL_HAPTIC_AUTOCENTER)
        {
            fgfswrite(sock, "get /haptic/device[%d]/autocenter", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.autocenter = atof(p);
        }

        if(devices[i].supported & SDL_HAPTIC_GAIN)
        {
            fgfswrite(sock, "get /haptic/device[%d]/gain", i);
            p = fgfsread(sock, TIMEOUT);
            if (p != NULL) devices[i].params.gain = atof(p);
        }

    }
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

    effectParams *oldParams = NULL;

    name = NULL;
    index = -1;

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

    // Connect to flightgear
    sock = fgfsconnect(DFLTHOST, DFLTPORT);
    if (sock < 0) {
        printf("Could not connect to flightgear!\n");
        return EXIT_FAILURE;
    }

    // Switch to data mode
    fgfswrite(sock, "data");

    // Initialize SDL haptics and send the devices to flightgear
    init_haptic();

    create_effects();

    // allocate memory for old param values
    oldParams = (effectParams *)malloc(num_devices * sizeof(effectParams));
    if(!oldParams) {
        printf("Fatal error: Could not allocate memory!\n");
        abort();
    }

    // Read and apply initial parameters
    read_fg();

    for(int i=0; i < num_devices; i++)
    {
        if(devices[i].supported & SDL_HAPTIC_AUTOCENTER) SDL_HapticSetAutocenter(devices[i].device, devices[i].params.autocenter * 100);
        if(devices[i].supported & SDL_HAPTIC_GAIN) SDL_HapticSetGain(devices[i].device, devices[i].params.gain * 100);
        if(devices[i].supported & SDL_HAPTIC_CONSTANT) SDL_HapticRunEffect(devices[i].device, devices[i].effectId[0], 1);
        if(devices[i].supported & SDL_HAPTIC_SINE) SDL_HapticRunEffect(devices[i].device, devices[i].effectId[1], 1);
    }


    // Main loop
    while(true)
    {
        // Back up old parameters
        for(int i=0; i < num_devices; i++)
            memcpy((void *)&oldParams[i], (void *)&devices[i].params, sizeof(effectParams));

        // Read new parameters
        printf("Read\n");
        read_fg();
        printf("Done\n");


        // If parameters have changed, apply them
        for(int i=0; i < num_devices; i++)
        {
            // Check for parameter changes
            // Pilot G force changed?
            if(devices[i].supported & SDL_HAPTIC_CONSTANT && (devices[i].params.pilot[0] != oldParams[i].pilot[0] || devices[i].params.pilot[1] != oldParams[i].pilot[1] ||
               devices[i].params.pilot[2] != oldParams[i].pilot[2] || devices[i].params.pilot_gain != oldParams[i].pilot_gain ||
               devices[i].params.surface[0] != oldParams[i].surface[0] || devices[i].params.surface[1] != oldParams[i].surface[1] || devices[i].params.surface[2] != oldParams[i].surface[2] ||
               devices[i].params.surface_gain != oldParams[i].surface_gain))
            {
//                if(SDL_HapticStopEffect(devices[i].device, devices[i].effectId[0]) < 0) printf("Error1 %s\n", SDL_GetError());
                SDL_HapticDestroyEffect(devices[i].device, devices[i].effectId[0]);

                float x = devices[i].params.pilot_gain*devices[i].params.pilot[0] + devices[i].params.surface_gain*devices[i].params.surface[0];
                float y = devices[i].params.pilot_gain*devices[i].params.pilot[1] + devices[i].params.surface_gain*devices[i].params.surface[1];
                float z = devices[i].params.pilot_gain*devices[i].params.pilot[2] + devices[i].params.surface_gain*devices[i].params.surface[2];

                float level = sqrt(x*x + y*y + z*z);

                devices[i].effect[0].constant.direction.dir[0] = (short)(x * 0x7FFF);
                devices[i].effect[0].constant.direction.dir[1] = (short)(y * 0x7FFF);
                devices[i].effect[0].constant.direction.dir[2] = (short)(z * 0x7FFF);
                devices[i].effect[0].constant.level = (short)(level * 0x7FFF);

//                if(SDL_HapticUpdateEffect(devices[i].device, devices[i].effectId[0], &devices[i].effect[0]) < 0) printf("Error2: %s\n", SDL_GetError());
                if((devices[i].effectId[0] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[0])) < 0) printf("Error (new): %s\n", SDL_GetError());
                if(SDL_HapticRunEffect(devices[i].device, devices[i].effectId[0], 1) < 0) printf("Error3: %s\n", SDL_GetError());
            }

            if(devices[i].supported & SDL_HAPTIC_SINE && (devices[i].params.shaker_dir != oldParams[i].shaker_dir ||
               devices[i].params.shaker_gain != oldParams[i].shaker_gain || devices[i].params.shaker_period != oldParams[i].shaker_period))
            {
                SDL_HapticDestroyEffect(devices[i].device, devices[i].effectId[1]);

                devices[i].effect[1].periodic.direction.dir[0] = devices[i].params.shaker_dir*100.0;

                devices[i].effect[1].periodic.length = 5000;  // Default 10 seconds?
                devices[i].effect[1].periodic.period = devices[i].params.shaker_period;    // 100 ms period = 10 Hz?
                devices[i].effect[1].periodic.magnitude = devices[i].params.shaker_gain * 0x7FFF;

                if((devices[i].effectId[1] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[1])) < 0) printf("Error (new): %s\n", SDL_GetError());
            }

            if(devices[i].supported & SDL_HAPTIC_SINE && devices[i].params.shaker_trigger && !oldParams[i].shaker_trigger)
            {
                if(SDL_HapticRunEffect(devices[i].device, devices[i].effectId[1], 1) < 0) printf("Error3: %s\n", SDL_GetError());
            }

            if(devices[i].supported & SDL_HAPTIC_AUTOCENTER && devices[i].params.autocenter != oldParams[i].autocenter) SDL_HapticSetAutocenter(devices[i].device, devices[i].params.autocenter * 100);
            if(devices[i].supported & SDL_HAPTIC_GAIN && devices[i].params.gain != oldParams[i].gain) SDL_HapticSetGain(devices[i].device, devices[i].params.gain * 100);
        }
    }

    SDL_Delay(10000);        /* Effects only have length 5000 */

    // Close flightgear connection
    fgfswrite(sock, "quit");
    fgfsclose(sock);


    if(oldParams) free(oldParams);

    // Close haptic devices
    for(int i=0; i < num_devices; i++)
      if(devices[i].open && devices[i].device) SDL_HapticClose(devices[i].device);
    if(devices) free(devices);

    /* Quit TODO: remove old */
    if (haptic != NULL)
        SDL_HapticClose(haptic);
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

    // Close flightgear connection
    fgfswrite(sock, "quit");
    fgfsclose(sock);

    // Close haptic devices
    for(int i=0; i < num_devices; i++)
      if(devices[i].open && devices[i].device) SDL_HapticClose(devices[i].device);
    if(devices) free(devices);


    if(haptic) SDL_HapticClose(haptic);
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
    if (supported & SDL_HAPTIC_SQUARE)
        printf("      square\n");
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
#ifdef USE_GENERIC
#else
        static char buf[MAXMSG];
        char *p;
        fd_set ready;
        struct timeval tv;
        ssize_t len;

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

        for (p = &buf[len - 1]; p >= buf; p--)
                if (*p != '\015' && *p != '\012')
                        break;
        *++p = '\0';

        //if(strlen(buf)) printf("%s\n", buf);
        return strlen(buf) ? buf : NULL;
#endif
}



void fgfsflush(int sock)
{
        const char *p;
        while ((p = fgfsread(sock, 0)) != NULL) {
                printf("IGNORE: \t<%s>\n", p);
        }
}

int fgfsconnect(const char *hostname, const int port)
{
        struct sockaddr_in serv_addr;
        struct hostent *hostinfo;
        int sock;

        sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
                perror("fgfsconnect/socket");
                return -1;
        }

        hostinfo = gethostbyname(hostname);
        if (hostinfo == NULL) {
                fprintf(stderr, "fgfsconnect: unknown host: \"%s\"\n", hostname);
                close(sock);
                return -2;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        serv_addr.sin_addr = *(struct in_addr *)hostinfo->h_addr_list[0];

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                perror("fgfsconnect/connect");
                close(sock);
                return -3;
        }
        return sock;
}

