//  Copyright (c) 2022 YCH, ATE, AJE, MHE
//  Developed as part of the line-following robot project at IUT de Cachan.

#include "IHM.h"
#include "NBoard.h"

// Function prototypes
void robotInit(void);
void lireCapteur(void);
float lireAN(int canal);
void automateBP(void);
void commandeMoteur(void);
void automateSuivi(void);

/**
 * Main loop of the line following robot
 * Continuously reads sensors, handles buttons, updates state machine
 * and controls motors
 */
int main() {
    robotInit();
    while(true) {
        lireCapteur();    // Read sensors
        automateBP();     // Handle button states
        automateSuivi();  // Update line following state machine
        commandeMoteur(); // Update motor commands
    }
}

/**
 * Initialize robot components:
 * - Set PWM period for both motors
 * - Reset motor values to 0
 * - Start chronometer
 * - Clear LED bar
 */
void robotInit(void) {
    MotD.period_us(50);  // Right motor PWM period
    MotG.period_us(50);  // Left motor PWM period
    MotD = 0;
    MotG = 0;
    chronometre.start();
    ihm.BAR_set(0);
}

/**
 * Read all sensors:
 * - Read potentiometer value for speed control
 * - Read all 5 line sensors through multiplexer
 * - Update LED display based on sensor values
 * - Update status LEDs for JACK and Button
 */
void lireCapteur(void) {
    static int numCapt[5] = {4, 3, 2, 1, 0};  // Sensor mapping
    int leds = 0;
    vpot = lireAN(7);  // Read potentiometer on channel 7
    for (int i = 0; i < 5; i++) {
        vcapteur[i] = lireAN(numCapt[i]);
        if (vcapteur[i] < vseuil) leds = leds + (1 << i);  // Light LED if line detected
    }
    bus5led = leds;    // Update sensor status LEDs
    Led6 = JACK;       // Display JACK status
    Led7 = BP;         // Display Button status
}

/**
 * Read analog value from specified channel through multiplexer
 * @param canal Channel number to read from
 * @return Analog value read (0.0 to 1.0)
 */
float lireAN(int canal) {
    BusSelectMux = canal;
    wait_us(1);        // Wait for multiplexer to settle
    return AnaIn.read();
}

/**
 * Button state machine handling:
 * - init: Display potentiometer value, wait for JACK to start
 * - run: Running state, check for stop button
 * - stop: Display elapsed time, wait for JACK to reset
 */
void automateBP(void) {
    static T_automBP etat = etat_init;
    switch (etat) {
        case etat_init:  // Initial state
            ihm.LCD_gotoxy(1, 0);
            ihm.LCD_printf("%5.3f", vpot);
            if (!JACK) {
                etat = etat_run;
                chronometre.reset();
                ihm.LCD_clear();
                run = 1;
            }
            break;

        case etat_run:   // Running state
            if (!BP) {
                etat = etat_stop;
                chronometre.stop();
                run = 0;
            }
            break;

        case etat_stop:  // Stopped state - display time
            ihm.LCD_gotoxy(1, 0);
            ihm.LCD_printf("%5.2f", chronometre.read());
            if (JACK) {
                etat = etat_init;
                ihm.LCD_clear();
                run = 0;
            }
            break;
    }
}

/**
 * Motor control function:
 * - When running: Apply speed from state machine multiplied by potentiometer value
 * - When stopped: Set motors to 0
 */
void commandeMoteur(void) {
    if (run) {
        MotD = vd * vpot;  // Right motor speed
        MotG = vg * vpot;  // Left motor speed
    } else {
        MotD = 0;
        MotG = 0;
    }
}

/**
 * Line following state machine
 * Uses geometric calculations for precise trajectory control
 * States:
 * - td: Straight line following
 * - corG/corD: Small corrections left/right
 * - virG/virD: Sharp turns left/right
 * - sorG/sorD: Exit from turns left/right
 */
void automateSuivi(void) {
    // Correction coefficients calculated from robot geometry
    static float coefcorr = sqrt((CORR-(VOIE/2))/(CORR+(VOIE/2)));  // Small corrections
    static float coef50 = sqrt((R50-(VOIE/2))/(R50+(VOIE/2)));      // 50cm radius turns
    static float coef30 = sqrt((R30-(VOIE/2))/(R30+(VOIE/2)));      // 30cm radius turns
    static T_automSuivi etat = etat_td;

    switch (etat) {
        case etat_td:  // Straight line state
            vg = 1.0f;
            vd = 1.0f;
            ihm.BAR_set(0x018);
            if (!CEG && !CG && CD && !CED) etat = etat_corD;        // Need right correction
            if (!CEG && CG && !CD && !CED) etat = etat_corG;        // Need left correction
            break;

        case etat_corG:  // Left correction
            vg = coefcorr;
            vd = 1.0f/coefcorr;
            ihm.BAR_set(0x020);
            if (!CEG && CG && CD && !CED) etat = etat_td;           // Back to center
            if (!CEG && !CG && CD && !CED) etat = etat_corD;        // Need right instead
            if (CEG && !CG && !CD && !CED) etat = etat_virG;        // Sharp left turn ahead
            break;

        case etat_corD:  // Right correction
            vd = coefcorr;
            vg = 1.0f/coefcorr;
            ihm.BAR_set(0x004);
            if (!CEG && CG && CD && !CED) etat = etat_td;           // Back to center
            if (!CEG && CG && !CD && !CED) etat = etat_corG;        // Need left instead
            if (!CEG && !CG && !CD && CED) etat = etat_virD;        // Sharp right turn ahead
            break;

        case etat_virG:  // Left turn
            vg = coef50;
            vd = 1.0f/coef50;
            ihm.BAR_set(0x040);
            if (!CEG && CG && !CD && !CED) etat = etat_corG;        // Exit turn phase 1
            if (!CEG && !CG && !CD && !CED) etat = etat_sorG;       // Exit turn phase 2
            break;

        case etat_virD:  // Right turn
            vd = coef50;
            vg = 1.0f/coef50;
            ihm.BAR_set(0x002);
            if (!CEG && !CG && CD && !CED) etat = etat_corD;        // Exit turn phase 1
            if (!CEG && !CG && !CD && !CED) etat = etat_sorD;       // Exit turn phase 2
            break;

        case etat_sorG:  // Left turn exit
            vg = coef30;
            vd = 1.0f/coef30;
            ihm.BAR_set(0x080);
            if (CEG && !CG && !CD && !CED) etat = etat_virG;        // Still in turn
            break;

        case etat_sorD:  // Right turn exit
            vd = coef30;
            vg = 1.0f/coef30;
            ihm.BAR_set(0x001);
            if (!CEG && !CG && !CD && CED) etat = etat_virD;        // Still in turn
            break;
    }
}
