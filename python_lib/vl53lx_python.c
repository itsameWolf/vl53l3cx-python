/*
MIT License
Copyright (c) 2022 Lupo Manes
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include "vl53lx_api.h"
#include "vl53lx_platform.h"

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 2

#define DISTANCEMODE_SHORT  1
#define DISTANCEMODE_MEDIUM 2
#define DISTANCEMODE_LONG   3

#define VL53LX_DEFAULT_ADDRESS 0x29

void print_pal_error(VL53LX_Error Status)
{
    char buf[VL53LX_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

/******************************************************************************
 * @brief   Initialises the device.
 *  @param  i2c_address - I2C Address to set for this device
 *  @param  TCA9548A_Device - Device number on TCA9548A I2C multiplexer if
 *              being used. If not being used, set to 255.
 *  @param  TCA9548A_Address - Address of TCA9548A I2C multiplexer if
 *              being used. If not being used, set to 0.
 * @retval  The Dev Object to pass to other library functions.
 *****************************************************************************/
VL53LX_Dev_t *initialise(uint8_t i2c_address)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    VL53LX_Version_t                   Version;
    VL53LX_Version_t                  *pVersion   = &Version;
    VL53LX_DeviceInfo_t                DeviceInfo;
    int32_t status_int;

    printf ("VL53LX Start Ranging Address 0x%02X\n\n", i2c_address);

    VL53LX_Dev_t *dev = (VL53LX_Dev_t *)malloc(sizeof(VL53LX_Dev_t));
    memset(dev, 0, sizeof(VL53LX_Dev_t));

    if (dev != NULL)
    {
        // Initialize Comms to the default address to start
        dev->I2cDevAddr      = i2c_address;

        VL53LX_Error VL53LX_DataInit(VL53LX_DEV Dev);
        
        VL53LX_init(dev);
        /*
         *  Get the version of the VL53LX API running in the firmware
         */

        // If the requested address is not the default, change it in the device
        if (i2c_address != VL53LX_DEFAULT_ADDRESS)
        {
            printf("Setting I2C Address to 0x%02X\n", i2c_address);
            // Address requested not default so set the address.
            // This assumes that the shutdown pin has been controlled
            // externally to this function.
            // The VL53LX API accepts an 8-bit i2c address (7-bits + the
            // R/W bit) so we must shift our address left one bit (multiply
            // it by two) to turn the 7-bit (eg 0x29) into 8-bit (0x52).
            Status = VL53LX_SetDeviceAddress(dev, (i2c_address * 2));
            dev->I2cDevAddr      = i2c_address;
        }

        if (Status == VL53LX_ERROR_NONE)
        {
            status_int = VL53LX_GetVersion(pVersion);
            if (status_int == 0)
            {
                /*
                 *  Verify the version of the VL53LX API running in the firmrware
                 */

                // Check the Api version. If it is not correct, put out a warning
                if( pVersion->major != VERSION_REQUIRED_MAJOR ||
                    pVersion->minor != VERSION_REQUIRED_MINOR ||
                    pVersion->build != VERSION_REQUIRED_BUILD )
                {
                    printf("VL53LX API Version Warning: Your firmware %d.%d.%d (revision %d). This requires %d.%d.%d.\n",
                        pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                        VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
                }
                // End of implementation specific

                Status = VL53LX_DataInit(dev); // Data initialization
                if(Status == VL53LX_ERROR_NONE)
                {
                    Status = VL53LX_GetDeviceInfo(dev, &DeviceInfo);
                    if(Status == VL53LX_ERROR_NONE)
                    {
                        printf("VL53LX_GetDeviceInfo:\n");
                        printf("Product Type : %d\n", DeviceInfo.ProductType);
                        printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
                        printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

                        if ((DeviceInfo.ProductRevisionMajor != 1) && (DeviceInfo.ProductRevisionMinor != 2)) {
                            printf("Error expected cut 1.2 but found cut %d.%d\n",
                                    DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                            Status = VL53LX_ERROR_NOT_SUPPORTED;
                        }
                    }

                    if(Status == VL53LX_ERROR_NONE)
                    {
                        Status = VL53LX_StaticInit(dev); // Device Initialization
                        // StaticInit will set interrupt by default

                        if(Status == VL53LX_ERROR_NONE)
                        {
                            Status = VL53LX_PerformRefCalibration(dev,
                                    &VhvSettings, &PhaseCal); // Device Initialization

                            if(Status == VL53LX_ERROR_NONE)
                            {
                                Status = VL53LX_PerformRefSpadManagement(dev,
                                        &refSpadCount, &isApertureSpads); // Device Initialization

                                if(Status != VL53LX_ERROR_NONE)
                                {
                                    printf ("Call of VL53LX_PerformRefSpadManagement\n");
                                }
                            }
                            else
                            {
                                printf ("Call of VL53LX_PerformRefCalibration\n");
                            }
                        }
                        else
                        {
                            printf ("Call of VL53LX_StaticInit\n");
                        }
                    }
                    else
                    {
                        printf ("Invalid Device Info\n");
                    }
                }
                else
                {
                    printf ("Call of VL53LX_DataInit\n");
                }
            }
            else
            {
                Status = VL53LX_ERROR_CONTROL_INTERFACE;
                printf("Call of VL53LX_GetVersion\n");
            }
        }
        else
        {
            printf("Call of VL53LX_SetAddress\n");
        }

        print_pal_error(Status);
    }
    else
    {
        printf("Memory allocation failure\n");
    }

    return dev;
}

/******************************************************************************
 * @brief   Start Ranging
 * @param   mode - ranging mode
 *              0 - Good Accuracy mode
 *              1 - Better Accuracy mode
 *              2 - Best Accuracy mode
 *              3 - Longe Range mode
 *              4 - High Speed mode
 * @note Mode Definitions
 *   Good Accuracy mode
 *       33 ms timing budget 1.2m range
 *   Better Accuracy mode
 *       66 ms timing budget 1.2m range
 *   Best Accuracy mode
 *       200 ms 1.2m range
 *   Long Range mode (indoor,darker conditions)
 *       33 ms timing budget 2m range
 *   High Speed Mode (decreased accuracy)
 *       20 ms timing budget 1.2m range
 * @retval  Error code, 0 for success.
 *****************************************************************************/
VL53LX_Error startRanging(VL53LX_Dev_t *dev, int mode)
{
    // Setup in continuous ranging mode
    VL53LX_Error Status = VL53LX_SetDeviceMode(dev, mode);

    if(Status == VL53LX_ERROR_NONE)
    {
      
        Status = VL53LX_StartMeasurement(dev)
    }
    else
    {
        printf ("Call of VL53LX_SetDeviceMode\n");
    }

    print_pal_error(Status);

    return Status;
}


/******************************************************************************
 * @brief   Get current distance in mm
 * @return  Current distance in mm or -1 on error
 *****************************************************************************/
/*
int32_t getDistance(VL53LX_Dev_t *dev)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t current_distance = -1;

    if (dev != NULL)
    {
        Status = WaitMeasurementDataReady(dev);

        if(Status == VL53LX_ERROR_NONE)
        {
            Status = VL53LX_GetRangingMeasurementData(dev,
                                pRangingMeasurementData);
            if(Status == VL53LX_ERROR_NONE)
            {
                current_distance = pRangingMeasurementData->RangeMilliMeter;
            }

            // Clear the interrupt
            VL53LX_ClearInterruptMask(dev,
                                VL53LX_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            // VL53LX_PollingDelay(dev);
        }
    }
    else
    {
        printf("Device not initialized\n");
    }

    return current_distance;
}
*/

/******************************************************************************
 * @brief   Stop Ranging
 *****************************************************************************/
void stopRanging(VL53LX_Dev_t *dev)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;

    printf ("Call of VL53LX_StopMeasurement\n");

    if (dev != NULL)
    {
        Status = VL53LX_StopMeasurement(dev);

        if(Status == VL53LX_ERROR_NONE)
        {
            printf ("Wait for stop to be completed\n");
            Status = WaitStopCompleted(dev);
        }

        if(Status == VL53LX_ERROR_NONE)
        {
            Status = VL53LX_ClearInterruptMask(dev,
                VL53LX_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        }

        print_pal_error(Status);

        free(dev);
        dev = NULL;
    }
    else
    {
        printf("Device not initialized\n");
    }
}
/*
static VL53LX_Dev_t        dev;
static VL53LX_DEV          Dev =  &dev;

int status;

VL53LX_Error VL53LX_DataInit(VL53LX_DEV Dev);


void RangingLoop(void)
{
  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady=0;
  int no_of_object_found=0,j;
  printf("Ranging loop starts\n");
  
  status = VL53LX_WaitDeviceBooted(Dev);
  status = VL53LX_DataInit(Dev);
  status = VL53LX_StartMeasurement(Dev);
  
  if(status){
    printf("VL53LX_StartMeasurement failed: error = %d \n", status);
    while(1);
  }
  
  do{ // polling mode
    status = VL53LX_GetMeasurementDataReady(Dev, &NewDataReady);                        
    usleep(250000); // 250 millisecond polling period, could be 1 millisecond.
    if((!status)&&(NewDataReady!=0)){
      status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      printf("Count=%5d, ", pMultiRangingData->StreamCount);
      printf("#Objs=%1d ", no_of_object_found);
      for(j=0;j<no_of_object_found;j++){
        if(j!=0)printf("\n                     ");
        printf("status=%d, D=%5dmm, S=%7dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
               pMultiRangingData->RangeData[j].RangeStatus,
               pMultiRangingData->RangeData[j].RangeMilliMeter,
               pMultiRangingData->RangeData[j].SigmaMilliMeter,
               pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
               pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
      }
      printf ("\n");
      if (status==0){
        status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
      }
    }
  }
  while (1);
}
*/