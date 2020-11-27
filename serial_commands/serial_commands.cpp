#include "serial_commands.h"

#ifndef USE_PC_SERIAL
    #define SERIAL pc
#else
    #define SERIAL s1
#endif

Thread serial_commands_thread;
static BufferedSerial s1(PA_9, PA_10, 115200);
static BufferedSerial pc(USBTX,USBRX, 115200);

void start_serial_commands() {
    serial_commands_thread.start(serial_commands_func);
}

void serial_commands_func() {
    char c[1];
    int MAX_COMMAND_LENGTH = 32;
    char cmd[MAX_COMMAND_LENGTH + 1];
    int pos = 0;

    char msg[] = "!!!\n";

    while(true) {
        while(SERIAL.readable()) {
            SERIAL.read(c, sizeof(c));
            //pc.write(c, sizeof(c));
            if (*c == ';' || *c == '\r') {
                cmd[pos] = '\0';
                InterpretCommand(cmd);
                pos = 0;
            } else {
                cmd[pos++] = *c;
            }
        }
    }
}

void InterpretCommand(char* cmd) {
    char c;
    float f;
    // Note: Putting a space in front of %c allows you type commands like:
    // f 2.0; l 0.01; h 0.08
    int num_parsed = sscanf(cmd, " %c %f", &c, &f);
    if (num_parsed < 1) {
        printf("Invalid command\n");
        return;
    }
    switch(c) {
        // Change gait frequency
        case 'f':
            printf("Set freq. to: %f\n", f);
            state_gait_params[state].freq = f;
            break;
        // Change stride length
        case 'l':
            printf("Set stride len to: %f\n", f);
            state_gait_params[state].step_length = f;
            break;
        // Change stride differential
        case 's':
            printf("Set step difference len to: %f\n", f);
            state_gait_params[state].step_diff = f;
            break;
        // Change stance height
        case 'h':
            printf("Set stance ht. to: %f\n", f);
            state_gait_params[state].stance_height = f;
            break;
        // Change gait up amplitude
        case 'u':
            printf("Set up amp. to: %f\n", f);
            state_gait_params[state].up_amp = f;
            break;
        // Change gait down amplitude
        case 'd':
            printf("Set down amp. to: %f\n", f);
            state_gait_params[state].down_amp = f;
            break;
        // Change gait flight percent
        case 'p':
            printf("Set flt. perc. to: %f\n", f);
            state_gait_params[state].flight_percent = f;
            break;
        case 'r':
            printf("Set straight dir. to: %f\n", f);
            break;
        case 't':
            printf("Set turn dir. to: %f\n", f);
            break;
        // Change leg gains
        // case 'g':
        //     { // Have to create a new scope here in order to declare variables
        //         float kp_t, kd_t, kp_g, kd_g;
        //         int res = sscanf(cmd, "g %f %f %f %f", &kp_t, &kd_t, &kp_g, &kd_g);
        //         if (res == 4) {
        //             Serial << "Set gains to: " << kp_t << " " << kd_t << " " << kp_g << " " << kd_g << "\n";
        //             gait_gains.kp_theta = kp_t;
        //             gait_gains.kd_theta = kd_t;
        //             gait_gains.kp_gamma = kp_g;
        //             gait_gains.kd_gamma = kd_g;
        //         } else {
        //             Serial.println("Invalid gain format.");
        //         }
        //     }
        //     break;
        // // Toggle debug printing
        case 'D':
            //enable_debug = !enable_debug;
            printf("Debug printing: \n");// << enable_debug << "\n";
            break;
        // Switch into STOP state
        case 'S':
            state = STOP;
            printf("STOP\n");
            break;
        // Switch into DANCE state
        case 'E':
            TransitionToDance();
            printf("DANCE\n");
            break;
        // Switch into BOUND state
        case 'B':
            TransitionToBound();
            printf("BOUND\n");
            break;
        // Switch into TROT state
        case 'T':
            TransitionToTrot();
            printf("TROT\n");
            break;
        // Swith into TURN_TROT
        case 'Y':
            TransitionToTurnTrot();
            printf("TURN TROT\n");
            break;
        // Switch into WALK state
        case 'W':
            TransitionToWalk();
            printf("WALK\n");
            break;
        // Switch into WALK state
        case 'P':
            TransitionToPronk();
            printf("PRONK\n");
            break;
        // Switch into JUMP state
        case 'J':
            //StartJump(millis()/1000.0f);
            printf("JUMP\n");
            break;
        case 'H':
            TransitionToHop();
            printf("HOP\n");
            break;
        case 'F':
            //StartFlip(millis()/1000.0f);
            printf("FLIP\n");
            break;
        case 'R':
            state = RESET_;
            printf("RESET\n");
            break;
        // // Switch into TEST state
        // TODO: Make new character for test mode
        // case '1':
        //     state = TEST;
        //     Serial.println("(1)TEST");
        //     break;
        default:
            printf("Unknown command\n");
    }
}

void PrintGaitCommands() {
    printf("Available gait commands:\n");
    printf("(f)req\n");
    printf("step (l)ength\n");
    printf("stance (h)eight\n");
    printf("(d)own amplitude\n");
    printf("(u)p amplitude\n");
    printf("flight (p)roportion\n");
    printf("(s)tep difference\n\n");
}