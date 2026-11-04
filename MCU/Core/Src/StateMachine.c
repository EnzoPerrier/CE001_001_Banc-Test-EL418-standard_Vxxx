/*
 * StateMachine.c
 *
 *  Created on: Jun 29, 2025
 *      Author: Enzo Perrier
 */
//---------------------------------------------------------------- Include
#include "main.h"
#include "cmsis_os.h"

#include "StateMachine.h"
#include "uart.h"
#include "ina219.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
//---------------------------------------------------------------- Define

#define MAX_PER_LENGTH 8
#define BAT_TOLERANCE_MV 200  // Ecart max entre mesure de tension MTI TEMPO et la valeur mesurée par l'INA219
#define CURRENT_MAX_MA  90  // Courant max accepté

//---------------------------------------------------------------- Variables

uint8_t state = 0;
char per_value[MAX_PER_LENGTH + 1];
static bool action_done = 0;

extern INA219_Data_t ina219_last;
extern volatile uint8_t ina219_valid;

//---------------------------------------------------------------- typedef

typedef struct
{ // Structure pour gérer la trame STS
    char ver[32];
    char crc[32];
    char lan[16];
    float acc;
    float bat;
    float cel_val;
    char cel_mode;
    char lum;
    bool dips[8]; // true = ON
    bool inps[3]; // true = ON
} TrameDataSTS;

//---------------------------------------------------------------- Prototypes fonctions
void parse_data_STS(char *buffer, TrameDataSTS *data);

void StateMachineTask(void)
{
    static bool bp_pressed = 0, bp3_pressed = 0, bp4_pressed = 0;

    //--------------------------- Vérification de l'appui sur BP3 pour revenir à l'étape précédente
    if (!HAL_GPIO_ReadPin(BP3_GPIO_Port, BP3_Pin) && state > 0 && !bp3_pressed)
    {
        // Si BP3 est pressé et qu'on n'est pas déjà revenu à l'étape précédente
        state--; // On revient à l'étape précédente
        action_done = 0;
        bp3_pressed = 1;
        osDelay(250);
    }
    else
    {
        bp3_pressed = 0;
    }
    //--------------------------- Vérification de l'appui sur BP4 pour reset (étape 0)
    if (!HAL_GPIO_ReadPin(BP4_GPIO_Port, BP4_Pin) && !bp4_pressed)
    {
        // Si BP3 est pressé et qu'on n'est pas déjà revenu à l'étape précédente
        state = 0; // On revient à l'étape 0
        action_done = 0;
        bp4_pressed = 1;
        osDelay(250);
    }
    else
    {
        bp4_pressed = 0;
    }

    //--------------------------- TRANSITIONS
    switch (state)
    {
    case 0:
    case 7:
    case 8:
    case 11:
    case 12:
    case 13:
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
            if (state < 13)
            {
                state++;
                osDelay(250);
            }
            else
            {
                state = 0;
                osDelay(250);
            }

            action_done = 0;
        }
        break;
    case 1:
        Check_UART1_Timeout(); // Permet de vérifier si on a fini de recevoir le message et retoure message_complete1

        if (message_complete1)
        {
            message_complete1 = 0;

            char expected[20];
            sprintf(expected, "PER = %s", per_value);

            if (strstr((char *)rx_buffer1, expected) != NULL)
            {
                send_UART3("PER VALIDE --> Etape suivante\r\n");
                // memset(rx_buffer1, 0, RX_BUFFER1_SIZE ); // On reset notre buffer par sécurité
                state++;
                action_done = 0;
            }
            else
            {
                send_UART3("Valeur differente. Entrez a nouveau le PER :\r\n");
            }
        }
        break;

    case 2: // Check STS
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            bool acc_ok = false;
            bool bat_ok = false;

            send_UART3((char *)rx_buffer1); // On a besoin de traiter la trame dans le logiciel de test

            // Vérification ACC
            if (data.acc >= 8.0 && data.acc <= 10.0)
            {
                // send_UART3("Accu OK\n"); // Utile pour test via terminal
                acc_ok = true;
            }
            else
            {
                send_UART3("STS NOK: Valeur ACCU hors plage !!\r\n");
            }

            // Vérification BAT : comparaison avec la tension mesurée par l'INA219 (± BAT_TOLERANCE_V)
            INA219_Data_t ina;
            uint8_t ina_ok;

            osKernelLock();              // copie cohérente (la tâche INA219 écrit la structure en parallèle)
            ina    = ina219_last;
            ina_ok = ina219_valid;
            osKernelUnlock();

            if (!ina_ok)
            {
                send_UART3("STS NOK: INA219 indisponible, verification BAT impossible !!\r\n");
            }
            else
            {
            	// --- Contrôle de la tension BAT par rapport à l'INA219 ---
            	int32_t bat_mV  = (int32_t)(data.bat * 1000.0f);
            	int32_t diff_mV = bat_mV - ina.bus_mV;
            	if (diff_mV < 0)
            	    diff_mV = -diff_mV;

            	if (diff_mV <= BAT_TOLERANCE_MV)
            	{
            	    bat_ok = true;
            	}
            	else
            	{
            	    char msg[90];
            	    snprintf(msg, sizeof(msg),
            	             "STS NOK: BAT=%ld mV, INA219=%ld mV (ecart > %d mV) !!\r\n",
            	             (long)bat_mV, (long)ina.bus_mV, BAT_TOLERANCE_MV);
            	    send_UART3(msg);
            	}

                // --- Contrôle du courant consommé ---
                int32_t i_uA = ina.current_uA;
                if (i_uA < 0)
                    i_uA = -i_uA;                     // valeur absolue (au cas où le shunt serait câblé à l'envers)

                if (i_uA > (int32_t)CURRENT_MAX_MA * 1000)
                {
                    int32_t ma10 = i_uA / 100;        // courant en 0.1 mA
                    char msg[70];
                    snprintf(msg, sizeof(msg),
                             "STS NOK: COURANT=%ld.%ld mA (> %d mA) !!\r\n",
                             (long)(ma10 / 10), (long)(ma10 % 10), CURRENT_MAX_MA);
                    send_UART3(msg);
                    bat_ok = false;                   // l'étape 2 échoue
                }
            }

            // Transition si tout est bon
            if (acc_ok && bat_ok)
            {
                osDelay(1500);
                send_UART3("STS OK --> Etape suivante\r\n");
                action_done = 0;
                state++;
            }
            else
            {
                send_UART3("STS invalide. Corrigez les erreurs ci-dessus et recommencez le  test\r\n");
            }
        }
        break;

    case 3: // DIPS OFF
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            bool dips_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Vérification DIP switches
            for (int i = 0; i < 8; i++)
            {
                if (data.dips[i])
                {
                    dips_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: DIP %d a ON !\r\n", i + 1); // On affiche l'état des DIPs en défaut
                    send_UART3(msg);
                }
                else if (!data.dips[i])
                {
                    char msg[50];
                    sprintf(msg, "OK: DIP %d a OFF \r\n", i + 1); // On affiche les DIPs OK
                    send_UART3(msg);
                }
            }

            if (dips_ok)
            {
                send_UART3("DIP a OFF --> OK\r\n");
                osDelay(1500);
                state++;
                bp_pressed = 0;
                action_done = 0;
                memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
                rx_index1 = 0;
            }
            else
            {
                send_UART3("DIP NOK : Tous les DIPs ne sont pas a OFF, mettez les a OFF et rappuyez sur le bouton valider\r\n");
                bp_pressed = 0;
                memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
                rx_index1 = 0;
            }
        }
        break;

    case 4: // DIPS ON
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            bool dips_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Vérification DIP switches
            for (int i = 0; i < 8; i++)
            {
                if (!data.dips[i])
                {
                    dips_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: DIP %d a OFF !\r\n", i + 1);
                    send_UART3(msg);
                }
                else if (data.dips[i])
                {
                    char msg[50];
                    sprintf(msg, "OK: DIP %d a ON \r\n", i + 1);
                    send_UART3(msg);
                }
            }

            if (dips_ok)
            {
                send_UART3("DIP a ON --> OK\r\n");
                osDelay(1500);
                state++;
                action_done = 0;
                bp_pressed = 0;
                // Nettoyer le buffer pour l'étape suivante
                memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
                rx_index1 = 0;
            }
            else
            {
                send_UART3("DIP NOK : Tous les DIPs ne sont pas a ON, mettez les a ON et rappuyez sur le bouton valider\r\n");
                // Nettoyer pour permettre une nouvelle tentative
                memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
                rx_index1 = 0;
            }
        }
        break;

    case 5: // Entrées à OFF
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;
            bool inps_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Verification entrées
            for (int i = 0; i < 3; i++)
            {
                if (data.inps[i])
                {
                    inps_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: IN %d a ON!\r\n", i + 1); // On affiche l'état des entrées
                    send_UART3(msg);
                }
                else if (!data.inps[i])
                {
                    char msg[50];
                    sprintf(msg, "OK: IN %d a OFF \r\n", i + 1); // On affiche les INPs OK
                    send_UART3(msg);
                }
            }

            if (inps_ok)
            {
                send_UART3("Entrees a OFF --> OK\r\n");
                osDelay(1500);
                state++;
                action_done = 0;
            }
        }
        break;

    case 6: // Test entrées à ON
        Check_UART1_Timeout();
        if (message_complete1)
        {
            message_complete1 = 0;

            bool inps_ok = true;

            TrameDataSTS data = {0};
            parse_data_STS((char *)rx_buffer1, &data);

            // Verification entrées
            for (int i = 0; i < 3; i++)
            {
                if (!data.inps[i])
                {
                    inps_ok = false;
                    char msg[50];
                    sprintf(msg, "ERROR: IN %d a OFF!\r\n", i + 1); // On affiche l'état des entrées
                    send_UART3(msg);
                }
                else if (data.inps[i])
                {
                    char msg[50];
                    sprintf(msg, "OK: IN %d a ON \r\n", i + 1); // On affiche les INPs OK
                    send_UART3(msg);
                }
            }

            if (inps_ok)
            {
                send_UART3("Entrees a ON --> OK\r\n");
                osDelay(1500);
                state++;
                action_done = 0;
            }
        }
        break;

    case 9: // Cellule JOUR
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
            Check_UART1_Timeout();
            if (message_complete1)
            {
                message_complete1 = 0;

                TrameDataSTS data = {0};
                parse_data_STS((char *)rx_buffer1, &data);

                if (data.cel_mode == 'J')
                {
                    send_UART3("Cellule JOUR --> OK\r\n");
                    osDelay(1500);
                    state++;
                    action_done = 0;
                }
                else
                {
                    send_UART3("ERROR: Defaut cellule en NUIT, veuillez reessayer\r\n");
                    action_done = 0;
                }
            }
        }
        break;

    case 10: // Cellule NUIT
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
            Check_UART1_Timeout();
            if (message_complete1)
            {
                message_complete1 = 0;

                TrameDataSTS data = {0};
                parse_data_STS((char *)rx_buffer1, &data);

                if (data.cel_mode == 'N')
                {
                    send_UART3("Cellule NUIT --> OK\r\n");
                    osDelay(1500);
                    state++;
                    action_done = 0;
                }
                else
                {
                    send_UART3("ERROR: Defaut cellule en JOUR, veuillez reessayer\r\n");
                    action_done = 0;
                }
            }
        }
        break;
    }

    //--------------------------- ACTIONS
    switch (state)
    {
    case 0:
        if (!action_done)
        {
            // On met bien les I/0 par défaut
            send_UART1("TST=0\r");
            HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_CEL_GPIO_Port, LED_CEL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);

            // Nettoyer tous les buffers UART
            memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
            memset(rx_buffer3, 0, RX_BUFFER3_SIZE); // <-- CRITIQUE !

            // Réinitialiser aussi per_value
            memset(per_value, 0, sizeof(per_value));

            send_UART3("---- ETAPE 0 ----\r\n");
            osDelay(10);
            //send_UART3("Appuyez sur le bouton valider pour commencer\r\n");
            action_done = 1;
        }
        break;
    case 1:
        if (!action_done)
        {
        	memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
        	message_complete1 = 0;
        	rx_index1 = 0;

            memset(rx_buffer3, 0, RX_BUFFER3_SIZE);

            send_UART3("---- ETAPE 1 ----\r\n");
            //send_UART3("Entrez le PER (juste la valeur sur 8 digits)\r\n");
            action_done = 1;
        }

        if (message_complete3)
        {
            message_complete3 = 0;
            // Nettoyer la chaîne des caractères \r et \n
            char cleaned_buffer[MAX_PER_LENGTH + 1] = {0}; // Initialise le buffer pour corriger le bug ("PER doit faire 8 digits")
            int i = 0;
            // Copier les caractères sauf \r et \n
            for (int j = 0; j < strlen((char *)rx_buffer3); j++)
            {
                if (rx_buffer3[j] != '\r' && rx_buffer3[j] != '\n')
                {
                    cleaned_buffer[i++] = rx_buffer3[j];
                }
            }
            cleaned_buffer[i] = '\0'; // Terminer la chaîne propre

            // Vérifier la longueur après nettoyage
            if (i == MAX_PER_LENGTH)
            {
                strncpy(per_value, cleaned_buffer, MAX_PER_LENGTH);

                char cmd[30];
                sprintf(cmd, "PER=%s\r", per_value);
                send_UART1(cmd);
                osDelay(10);
                send_UART1(cmd);
                send_UART3("PER envoye. Attente confirmation…\r\n");
                osDelay(1000);
                send_UART1("PER\r");
                osDelay(50);
            }
            else
            {
                send_UART3("Format invalide: Le PER doit faire 8 digits, recommencez…\r\n");
            }
        }
        break;

    case 2:
        if (!action_done)
        {
            send_UART3("---- ETAPE 2 ----\r\n");
            //send_UART3("Test STS en cours ...\r\n");
            osDelay(500);
            send_UART1("STS\r");
            osDelay(500);
            send_UART3((char *)rx_buffer1);
            action_done = 1;
        }
        break;

    case 3: // DIPs OFF
        if (!action_done)
        {
            send_UART3("---- ETAPE 3 ----\r\n");
            //send_UART3("Mettez les DIPs a OFF et appuyez sur le bouton valider\r\n");
            osDelay(500);
            action_done = 1;
        }
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) && !bp_pressed)
        {
            bp_pressed = 1;
            memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
            rx_index1 = 0;
            message_complete1 = 0;
            send_UART1("STS\r");
        }
        // Réinitialiser bp_pressed quand BP2 est relâché
        if (HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
            bp_pressed = 0;
        }
        break;

    case 4: // DIPs ON
        if (!action_done)
        {
            send_UART3("---- ETAPE 4 ----\r\n");
            //send_UART3("Mettez les DIPs a ON, appuyez sur le BP reset et appuyez sur le bouton valider\r\n");
            osDelay(500);
            action_done = 1;
        }
        if (!HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin) && !bp_pressed)
        {
            bp_pressed = 1;
            memset(rx_buffer1, 0, RX_BUFFER1_SIZE); // Nettoyer le buffer avant l'envoi
            rx_index1 = 0;
            message_complete1 = 0;
            send_UART1("STS\r");
        }
        // Réinitialiser bp_pressed quand BP2 est relâché
        if (HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin))
        {
            bp_pressed = 0;
        }
        break;
        break;

    case 5: // Entrées à OFF
        if (!action_done)
        {
            send_UART3("---- ETAPE 5 ----\r\n");
            //send_UART3("Test entrees à OFF en cours...\r\n");
            osDelay(500);
            // Désactivation de toutes les entrées
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
            osDelay(800);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;

    case 6: // ENtrées  à  ON
        if (!action_done)
        {
            send_UART1("TST=0\r");
            send_UART3("---- ETAPE 6 ----\r\n");
            osDelay(500);
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
            osDelay(800);
            // Nettoyage du buffer avant envoi STS
            memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
            rx_index1 = 0;
            message_complete1 = 0;
            send_UART1("STS\r");
            action_done = 1;
        }
        break;

    case 7: // Test décompteur
        if (!action_done)
        {
            // On réinitialise les entrées précédemment sur ON
            HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
            send_UART3("---- ETAPE 7 ----\r\n");
            //send_UART3("Test du decompteur...\n Veuillez valider en appuyant sur le bouton valider si toutes les leds s'allument correctement et dans le bon ordre sur le decompteur\n\r");
            send_UART1("TST=1\r");
            osDelay(1500); // Délai pour éviter de passer trop vite à l'étape suivante
            action_done = 1;
        }
        break;

    case 8: // Défauts ampoules
        if (!action_done)
        {
            HAL_GPIO_WritePin(LED_CEL_GPIO_Port, LED_CEL_Pin, GPIO_PIN_RESET);
            send_UART1("TST=0\r"); // On arrête le test décompteur
            send_UART3("---- ETAPE 8 ----\r\n");
            //send_UART3("Test des ampoules ...\n\r Verifiez que les ampoules s'eteignent et se rallument et que le defaut sur l'écran LCD de la carte corresponde bien a la bonne optique\n\rEnsuite appuyez sur le bouton valider\r\n");
            osDelay(1250);
            send_UART3("OPTR\r\n"); // Utile pour l'animation sur le logiciel AppTest418
            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
            send_UART3("OPTY\r\n");
            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
            send_UART3("OPTG\r\n");
            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
            osDelay(2000);
            send_UART3("OPTFULL\r\n");
            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
            action_done = 1;
        }
        break;
    case 9: // Test cellule Jour
        // on remet les relais à l'état initial
        if (!action_done)
        {
            HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);
            send_UART3("---- ETAPE 9 ----\r\n");
            //send_UART3("Test cellule JOUR, veuillez exposer la cellule a la lumiere\r\nappuyez sur le bouton valider pour tester\r\n");
            osDelay(1000);
            HAL_GPIO_WritePin(LED_CEL_GPIO_Port, LED_CEL_Pin, GPIO_PIN_SET);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;

    case 10: // Test cellule Nuit
        if (!action_done)
        {
            HAL_GPIO_WritePin(LED_CEL_GPIO_Port, LED_CEL_Pin, GPIO_PIN_RESET);
            send_UART3("---- ETAPE 10 ----\r\n");
            //send_UART3("Test cellule NUIT, veuillez cacher la cellule et reset la carte\r\nappuyez sur le bouton valider pour tester\n\r");
            osDelay(1000);
            send_UART1("STS\r");
            action_done = 1;
        }
        break;

    case 11: // Test IR
        if (!action_done)
        {
            HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_RESET);
            osDelay(1000);
            send_UART3("---- ETAPE 11 ----\r\n");
            //send_UART3("Test de l'infrarouge...\n Veuillez valider en appuyant sur le bouton valider si la telecommande fonctionne en emission et reception\r\n");
            action_done = 1;
        }
        break;

    case 12: // Test Accu
        if (!action_done)
        {
            osDelay(1000);
            send_UART3("---- ETAPE 12 ----\r\n");
            HAL_GPIO_WritePin(RELAIS_ALIM_418_GPIO_Port, RELAIS_ALIM_418_Pin, GPIO_PIN_SET);
            //send_UART3("Test de l'accu...\n Veuillez verifier que vous avez bien le message suppression batterie qui s'affiche à l'ecran, si le cas appuyez sur le bouton valider\r\n");
            action_done = 1;
        }
        break;

    case 13: // Test carte s'éteint après appui long sur  IN1
            if (!action_done)
            {
                osDelay(1000);
                send_UART3("---- ETAPE 13 ----\r\n");
                HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
                //send_UART3("Test extinction de la carte à l'appui de 5s sur IN1\r\n");
                osDelay(5750);
                HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
                action_done = 1;
            }
            break;

    default:
        break;
    }
}

//---------------------------------------------------------------- Fonctions outils
void parse_data_STS(char *buffer, TrameDataSTS *data)

{

    char *line = strtok(buffer, "\r\n");

    bool cel_val_parsed = false;

    bool cel_mode_parsed = false;

    while (line != NULL)

    {

        if (strncmp(line, "VER =", 5) == 0)

        {

            sscanf(line, "VER = %31[^\r\n]", data->ver);
        }

        else if (strncmp(line, "CRC =", 5) == 0)

        {

            sscanf(line, "CRC = %63[^\r\n]", data->crc);
        }

        else if (strncmp(line, "LAN =", 5) == 0)

        {

            sscanf(line, "LAN = %15[^\r\n]", data->lan);
        }

        else if (strncmp(line, "ACC =", 5) == 0)

        {

            sscanf(line, "ACC = %f", &data->acc);
        }

        else if (strncmp(line, "BAT =", 5) == 0)

        {

            sscanf(line, "BAT = %f", &data->bat);
        }

        else if (strncmp(line, "CEL =", 5) == 0)

        {

            // Essaye de parser float si pas encore fait

            if (!cel_val_parsed && sscanf(line, "CEL = %f", &data->cel_val) == 1)

            {

                cel_val_parsed = true;
            }

            // Sinon essaye de parser mode char, indépendamment

            else if (!cel_mode_parsed && sscanf(line, "CEL = %c", &data->cel_mode) == 1)

            {

                cel_mode_parsed = true;
            }
        }

        else if (strncmp(line, "LUM =", 5) == 0)

        {

            sscanf(line, "LUM = %c", &data->lum);
        }

        else if (strncmp(line, "DIP =", 5) == 0)

        {

            char *p = line + 5;

            int num;

            char state[4];

            while (sscanf(p, " %d:%3s", &num, state) == 2)

            {

                if (num >= 1 && num <= 8)

                    data->dips[num - 1] = (strcmp(state, "ON") == 0);

                char *next = strchr(p, ' ');

                if (!next)
                    break;

                p = next + 1;
            }
        }

        else if (strncmp(line, "INP =", 5) == 0)

        {

            char *p = line + 5;

            int num;

            char state[4];

            while (sscanf(p, " %d:%3s", &num, state) == 2)

            {

                if (num >= 1 && num <= 3)

                    data->inps[num - 1] = (strcmp(state, "ON") == 0);

                char *next = strchr(p, ' ');

                if (!next)
                    break;

                p = next + 1;
            }
        }

        line = strtok(NULL, "\r\n");
    }
}


// Appelée à la sortie du mode stream : retour à l'étape 0
void StateMachine_Resume(void)
{
    state = 0;
    action_done = 0;              // l'étape 0 rejouera son initialisation
    memset(rx_buffer1, 0, RX_BUFFER1_SIZE);
    rx_index1 = 0;
    message_complete1 = 0;
}
