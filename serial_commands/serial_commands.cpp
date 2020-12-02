#include "serial_commands.h"


#if defined(USE_PC_SERIAL)
    #define SERIAL pc
#else
    #define SERIAL s1
#endif

Thread serial_commands_thread;
static BufferedSerial s1(PA_9, PA_10, 115200);
static BufferedSerial pc(USBTX,USBRX, 115200);

void start_serial_commands(struct LegIdentifier* legs) {
    serial_commands_thread.start(serial_commands_func);
    legs_ = legs;
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
            //Spc.write(c, sizeof(c));
            if (*c == ';' || *c == '\n' || *c == '\r') {
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
    char s;
    float f;
    States temp_state = STOP;
    // Note: Putting a space in front of %c allows you type commands like:
    // f 2.0; l 0.01; h 0.08
    int num_parsed = sscanf(cmd, " %c %c %f", &c, &s, &f);
    printf("%s", cmd);
    printf("\n");
    printf("%c %c %f\n", c, s, f);
    if (num_parsed < 1) {
        printf("Invalid command\n");
        return;
    }
    switch(s) {
        // Change DANCE state params
        case 'E':
        temp_state = DANCE;
            break;
        // Change BOUND state params
        case 'B':
            temp_state = BOUND;
            break;
        // Change TROT state params
        case 'T':
            temp_state = TROT;
            break;
        // Change TURN_TROT state params
        case 'Y':
            temp_state = TURN_TROT;
            break;
        // Change WALK state params
        case 'W':
            temp_state = WALK;
            break;
        // Change PRONK state params
        case 'P':
            temp_state = PRONK;
            break;
        // Change HOP state params
        case 'H':
            temp_state = HOP;
            break;
        // Change FLIP state params
        case 'F':
            temp_state = FLIP;
            break;
        default:
            printf("Invalid or non-modifiable state\n");
    }
    switch(c) {
        // Change gait frequency
        case 'f':
            printf("Set state %d freq. to: %f\n", temp_state, f);
            state_gait_params[temp_state].freq = f;
            break;
        // Change stride length
        case 'l':
            printf("Set state %d stride len to: %f\n", temp_state, f);
            state_gait_params[temp_state].step_length = f;
            break;
        // Change stride differential
        case 's':
            printf("Set state %d step difference len to: %f\n", temp_state, f);
            state_gait_params[temp_state].step_diff = f;
            break;
        // Change stance height
        case 'h':
            printf("Set state %d stance ht. to: %f\n", temp_state, f);
            state_gait_params[temp_state].stance_height = f;
            break;
        // Change gait up amplitude
        case 'u':
            printf("Set state %d up amp. to: %f\n", temp_state, f);
            state_gait_params[temp_state].up_amp = f;
            break;
        // Change gait down amplitude
        case 'd':
            printf("Set state %d down amp. to: %f\n", temp_state, f);
            state_gait_params[temp_state].down_amp = f;
            break;
        // Change gait flight percent
        case 'p':
            printf("Set state %d flt. perc. to: %f\n", temp_state, f);
            state_gait_params[temp_state].flight_percent = f;
            break;
        case 'r':
            for (int i = 0; i < 4; i++) legs_[i].straight_dir = (int) f;
            printf("Set state %d straight dir. to: %f\n", temp_state, f);
            break;
        case 't':
            legs_[0].straight_dir = (int) f;
            legs_[1].straight_dir = (int) f;
            legs_[2].straight_dir = (int) -f;
            legs_[3].straight_dir = (int) -f;
            printf("Set state %d turn dir. to: %f\n", temp_state, f);
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