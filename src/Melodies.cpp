
#include <Arduino.h>
#include "Melodies.h"

////////////////////////////////////////////////////////////////////////
//
//                    GAME OF THRONES THEME SONG
//                PERFORMED ON PIEZO SENSOR AND 3 LEDS
//                           WRITTEN BY
//                           KALEB HILL
//                            5/28/15
//
////////////////////////////////////////////////////////////////////////

// Declaración externa: enlaza esta librería con el programa principal
extern volatile bool melodyOn;
extern TaskHandle_t playMelodyTaskHandle;

const static int buzzFrequency = 5000;
const static int buzzResolution = 12;

static inline bool isMelodyTask()
{
    return (playMelodyTaskHandle != NULL && xTaskGetCurrentTaskHandle() == playMelodyTaskHandle);
}

static inline bool isMelodyTaskAndOff()
{
    return (isMelodyTask() && !melodyOn);
}

void reattachBuzzer(uint32_t buzzPin, uint8_t buzzChannel)
{
    ledcSetup(buzzChannel, buzzFrequency, buzzResolution);
    ledcAttachPin(buzzPin, buzzChannel);
    ledcWriteTone(buzzChannel, 0);
}

void toneToPlay(uint32_t buzzPin, uint8_t buzzChannel, uint32_t buzzNote, uint32_t buzzDuration)
{
    if (isMelodyTaskAndOff())
        return;

    ledcSetup(buzzChannel, buzzFrequency, buzzResolution);
    ledcAttachPin(buzzPin, buzzChannel);
    ledcWriteTone(buzzChannel, buzzNote);

    if (isMelodyTask())
    {
        // Espera interrumpible al instante al apagar la melodía
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(buzzDuration)) > 0 || !melodyOn)
        {
            ledcWriteTone(buzzChannel, 0);
            return;
        }
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(buzzDuration));
    }

    ledcWriteTone(buzzChannel, 0); // Silenciamos sin destruir el timer
}

void toneToPlay(uint32_t buzzPin, uint8_t buzzChannel, uint32_t buzzNote, uint32_t buzzDuration, uint32_t buzzBips)
{
    if (isMelodyTaskAndOff())
        return;

    for (uint32_t i = 0; i < buzzBips; i++)
    {
        if (isMelodyTaskAndOff())
            return;

        ledcSetup(buzzChannel, buzzFrequency, buzzResolution);
        ledcAttachPin(buzzPin, buzzChannel);
        ledcWriteTone(buzzChannel, buzzNote);

        if (isMelodyTask())
        {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(buzzDuration)) > 0 || !melodyOn)
            {
                ledcWriteTone(buzzChannel, 0);
                return;
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(buzzDuration));
        }

        ledcWriteTone(buzzChannel, 0);
    }
}

void gameOfThrones(uint32_t buzzPin, uint8_t buzzChannel)
{
    ////////////////////////////////INTRO/////////////////////////////////////////////////////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_G4, 375);

    for (int i = 0; i < 4; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G4, 375);
    }

    for (int i = 0; i < 4; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_E4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G4, 375);
    }

    /////////////////////////////////////////////////CHORUS1 & 2////////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_G4, 1175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 1175);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G4, 775);
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 775);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F4, 175);

        //////////////////////////////

        for (int i = 0; i < 4; i++)
        {
            toneToPlay(buzzPin, buzzChannel, NOTE_D4, 375);
            toneToPlay(buzzPin, buzzChannel, NOTE_G3, 375);
            toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 175);
            toneToPlay(buzzPin, buzzChannel, NOTE_C4, 175);
        }

        //////////////////////////////
        toneToPlay(buzzPin, buzzChannel, NOTE_F4, 1175);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 1175);
        toneToPlay(buzzPin, buzzChannel, NOTE_D4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F4, 775);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 775);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_D4, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 500);

        //////////////////////////////
        for (int i = 0; i < 4; i++)
        {
            toneToPlay(buzzPin, buzzChannel, NOTE_GS3, 175);
            toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 175);
            toneToPlay(buzzPin, buzzChannel, NOTE_C4, 375);
            toneToPlay(buzzPin, buzzChannel, NOTE_F3, 375);
        }
    }

    /////////////////////////////////////////////////CHORUS3////////////////////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_G5, 1175);
    toneToPlay(buzzPin, buzzChannel, NOTE_C5, 1175);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS5, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_F5, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_G5, 775);
    toneToPlay(buzzPin, buzzChannel, NOTE_C5, 775);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS5, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_F5, 175);

    //////////////////////////////

    for (int i = 0; i < 4; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_D4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_G3, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 175);
    }

    //////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_F5, 1175);
    toneToPlay(buzzPin, buzzChannel, NOTE_AS4, 1175);
    toneToPlay(buzzPin, buzzChannel, NOTE_D5, 575);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS5, 575);
    toneToPlay(buzzPin, buzzChannel, NOTE_D5, 575);
    toneToPlay(buzzPin, buzzChannel, NOTE_AS4, 575);

    //////////////////////////////

    for (int i = 0; i < 4; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_GS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 375);
    }

    //////////////////////////////////////ENDING PART 1////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_C5, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G3, 375);
    }

    ///////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_AS4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_D3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 375);
    }

    //////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_GS4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_C3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_D3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 375);
    }

    /////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_G4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS2, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_D3, 375);
    }

    /////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_G2, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_GS2, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS2, 375);
    }

    //////////////////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 375);
    toneToPlay(buzzPin, buzzChannel, NOTE_G2, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_G2, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 375);
    toneToPlay(buzzPin, buzzChannel, NOTE_F4, 775);
    toneToPlay(buzzPin, buzzChannel, NOTE_GS2, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_GS2, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_F4, 375);

    //////////////////////////////////////////

    for (int i = 0; i < 4; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_GS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 375);
    }

    //////////////////////////////////////ENDING PART 2////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_C5, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G3, 375);
    }

    ///////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_AS4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_D3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 375);
    }

    //////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_GS4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_C3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_D3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 375);
    }

    /////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_G4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS2, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_C3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_D3, 375);
    }

    /////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_G2, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_GS2, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_AS2, 375);
    }

    //////////////////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 775);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS4, 375);
    toneToPlay(buzzPin, buzzChannel, NOTE_D4, 775);
    toneToPlay(buzzPin, buzzChannel, NOTE_D4, 375);

    //////////////////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_C4, 375);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_F3, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_G3, 375);

    ///////////

    for (int i = 0; i < 4; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_C3, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F3, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G3, 375);
    }
    ////////////////////////////////////////////////

    toneToPlay(buzzPin, buzzChannel, NOTE_C3, 375);
    toneToPlay(buzzPin, buzzChannel, NOTE_DS5, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_F5, 175);
    toneToPlay(buzzPin, buzzChannel, NOTE_G5, 375);

    /////////////////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        toneToPlay(buzzPin, buzzChannel, NOTE_C5, 375);
        toneToPlay(buzzPin, buzzChannel, NOTE_DS5, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_F5, 175);
        toneToPlay(buzzPin, buzzChannel, NOTE_G5, 375);
    }
}
