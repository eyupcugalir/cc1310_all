  // XDCtools Header files //
  #include <xdc/std.h>
  #include <xdc/runtime/System.h>
  #include <xdc/cfg/global.h>

  // BIOS Header files //
  #include <ti/sysbios/BIOS.h>
  #include <ti/sysbios/knl/Task.h>
  #include <ti/sysbios/knl/Clock.h>


  // TI-RTOS Header files //
  #include <ti/drivers/PIN.h>
  #include <ti/drivers/UART.h>
  #include <ti/drivers/I2C.h>
  #include <ti/drivers/SPI.h>
  #include <ti/drivers/rf/RF.h>

  // Example/Board Header files
  #include "Board.h"

  #include "ext_flash.h"
  #include "bsp_spi.h"

  #include "smartrf_settings.h"



  #include <stdlib.h>
  #include <stdbool.h>
  #include <stdint.h>
  #include <string.h>

  static PIN_Handle ledPinHandle;
  static PIN_State ledPinState;
  PIN_Config ledPinTable[] = {
      Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	 // Board_CP2103UARTRESET   | PIN_GPIO_OUTPUT_EN	| PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX,

      PIN_TERMINATE
  };


  #define TX_TASK_STACK_SIZE 1024
  #define TX_TASK_PRIORITY   2
  #define TASKSTACKSIZE    1024
  #define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )

	/* TX Configuration */
  #define PAYLOAD_LENGTH      30
  #define PACKET_INTERVAL     (uint32_t)(4000000*0.5f) /* Set packet interval to 500ms */

  static Task_Params txTaskParams;
  Task_Struct txTask;    /* not static so you can see in ROV */
  static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

  static RF_Object rfObject;
  static RF_Handle rfHandle;
  void taskFxn(UArg arg0, UArg arg1);

  uint32_t time;
  static uint8_t packet[PAYLOAD_LENGTH];
  static uint16_t seqNumber;
/*
  static PIN_Handle pinHandle;

  void TxTask_init(PIN_Handle inPinHandle)
  {
      pinHandle = inPinHandle;

      Task_Params_init(&txTaskParams);
      txTaskParams.stackSize = TX_TASK_STACK_SIZE;
      txTaskParams.priority = TX_TASK_PRIORITY;
      txTaskParams.stack = &txTaskStack;
      txTaskParams.arg0 = (UInt)1000000;

      Task_construct(&txTask, (Task_FuncPtr)taskFxn, &txTaskParams, NULL);
  }
*/



  Task_Struct task0Struct;
  Char task0Stack[TASKSTACKSIZE];

  // Global memory storage for a PIN_Config table
  //static SPI_Handle spiHandle;
 // static SPI_Params spiParams;

  //
  // Application LED pin configuration table:
  //   - All LEDs board LEDs are off.
   //



  Void taskFxn(UArg arg0, UArg arg1)
  {

	  //  ======== I2C PART ========
	  unsigned int    j;
	  uint16_t        temperature;
	  //uint8_t         txBuffer[2];
	  uint8_t         rxBuffer[2];
	  I2C_Handle      i2c;
	  I2C_Params      i2cParams;
	  I2C_Transaction i2cTransaction;

	  uint8_t         txbuf_threshold[3];

	  /* Create I2C for usage */
	  I2C_Params_init(&i2cParams);
	  i2cParams.bitRate = I2C_100kHz;
	  i2c = I2C_open(Board_I2C_TMP, &i2cParams);
	  if (i2c == NULL) {
		  System_abort("\nError Initializing I2C\n");
	  }
	  else {
		  System_printf("\nI2C Initialized!\n");
		  System_flush();
	  }



	  txbuf_threshold[0] = 0x00;
	  i2cTransaction.slaveAddress = Board_TMP112_ADDR;
	  i2cTransaction.writeBuf = txbuf_threshold;
	  i2cTransaction.writeCount = 1;
	  i2cTransaction.readBuf = rxBuffer;
	  i2cTransaction.readCount = 2;
	  for (j = 0; j < 4; j++) {
		  if (I2C_transfer(i2c, &i2cTransaction)) {


			  // System_printf("Sample %d (C)\n", rxBuffer[0]);
			  //System_flush();

			  //System_printf("Sample %d (C)\n", rxBuffer[1]);
			  //System_flush();

			  temperature = (rxBuffer[0] << 4) | (rxBuffer[1] >> 4);
			  temperature /= 16;
			  System_printf("Sample %u: %d (C)\n", j, temperature);
			  System_flush();


		  }
		  else {
			  System_printf("I2C Bus fault\n");
			  System_flush();

		  }

		  System_flush();
		  delay_ms(1000);
	  }
	  /* Deinitialized I2C */
	  I2C_close(i2c);
	  System_printf("I2C closed!\n");
	  System_flush();






	  //  ======== SPI PART ========
	  uint8_t id[256] ;
	  uint32_t  offset=0x0000;
	  uint8_t readid[256];
	  uint8_t page1 =0x01;
	  int i,y;
	  // Application main loop
	   for (y=0; y<2; y++)
	   {

		   for ( i=0; i<256;i++)
		   {
			   id[i]=i;
		   }


		/*   if (extFlashTest())
		   {
			   System_printf("success\n");
			   System_flush();
		   }else {
			   System_printf("nooooo\n");
			   System_flush();
		   }
		   delay_ms(2000);*/

           memset(readid, 5, sizeof(readid));
          // System_printf( "id =  %x %x %x %x %x %x %x %x\n", id[0],id[1],id[2],id[3],id[4],id[5],id[6],id[7]);
        	//	   System_flush();
           extFlashOpen();

           	   OADTarget_writeFlash(page1, offset, id, sizeof(id));
        	   delay_ms(2000);
        	   OADTarget_readFlash(page1, offset, readid, sizeof(readid));
        	   System_printf( "id = %x %x %x %x %x %x %x %x %x %x %x  %x \n", readid[0],readid[1],readid[42],readid[33],readid[44],readid[5],readid[6],readid[7],readid[8],readid[9],readid[10],readid[11]);
        	   System_flush();
        	   delay_ms(2000);
        	   OADTarget_eraseFlash(page1);

        	   delay_ms(2000);
        	   OADTarget_readFlash(page1, offset, readid, sizeof(readid));

        	   System_printf( "sil id = %x %x %x %x %x %x %x %x %x %x %x  %x \n", readid[0],readid[1],readid[2],readid[3],readid[4],readid[5],readid[6],readid[7],readid[8],readid[9],readid[10],readid[11]);
        	   System_flush();

		   //extFlashReadID(id);
         /* if(extFlashWrite(offset, sizeof (id), id))
          {
        	  delay_ms(2000);
        	  extFlashRead(offset, sizeof (readid), readid);
        	  System_printf( "id = %x %x %x %x %x %x %x %x %x %x %x  %x \n", readid[0],readid[1],readid[2],readid[3],
        			  readid[4],readid[5],readid[6],readid[7],readid[8],readid[9],readid[10],readid[11]);
        	  System_flush();
        	  delay_ms(2000);
          }
          else{
        	  System_printf( "Yazmadı\n");
        	  System_flush();
          }*/
		   extFlashClose();

	   }




	   //  ======== RF PART ========

	    uint32_t time;
	    RF_Params rfParams;
	    RF_Params_init(&rfParams);

	    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
	    RF_cmdPropTx.pPkt = packet;
	    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
	    RF_cmdPropTx.startTrigger.pastTrig = 1;
	    RF_cmdPropTx.startTime = 0;

	     //Request access to the radio
	    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

	     //Set the frequency
	    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

	    // Get current time
	    time = RF_getCurrentTime();
	    while(1)
	    {
	      //   Create packet with incrementing sequence number and random payload
	        packet[0] = (uint8_t)(seqNumber >> 8);
	        packet[1] = (uint8_t)(seqNumber++);
	        uint8_t i;
	        for (i = 2; i < PAYLOAD_LENGTH; i++)
	        {
	            packet[i] = rand();
	        }

	       //  Set absolute TX time to utilize automatic power management
	        time += PACKET_INTERVAL;
	        RF_cmdPropTx.startTime = time;

	        // Send packet
	        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
	        if (RF_EventTxDone)
	        {
	        	PIN_setOutputValue(ledPinHandle, Board_LED2, !PIN_getOutputValue(Board_LED2));
	        }

	        if (!(result & RF_EventLastCmdDone))
	        {
	          //   Error
	            while(1);
	        }

	       // PIN_setOutputValue(pinHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));
	    }


	   //  ======== UART PART ========
	   int k;
	   char input;
	   UART_Handle uart;
	   UART_Params uartParams;
	   const char echoPrompt[] = "\nEchoing characters:\n";

	   // Create a UART with data processing off.
	   UART_Params_init(&uartParams);
	   uartParams.writeDataMode = UART_DATA_BINARY;
	   uartParams.readDataMode = UART_DATA_BINARY;
	   uartParams.readReturnMode = UART_RETURN_FULL;
	   uartParams.readEcho = UART_ECHO_OFF;
	   uartParams.baudRate = 9600;
	   uart = UART_open(Board_UART0, &uartParams);

	   if (uart == NULL) {
		   System_abort("\nError opening the UART");
		   System_flush();
	   }else{
		   System_printf("\nUART OPEN !\n");
		   System_flush();
	   }

	   UART_write(uart, echoPrompt, sizeof(echoPrompt));

	   k=0;
	   while (k<10) {
		   UART_read(uart, &input, 1);
		   UART_write(uart, &input, 1);
		   System_printf(" %c " , input);
		   System_flush();
		   k++;

	   }
	   UART_close(uart);
	   if (uart == NULL) {
		   System_abort("\nError closing the UART");
		   System_flush();
	   }else{
		   System_printf("\nUART CLOSED !\n");
		   System_flush();
	   }



  }



   //  ======== main ========

  int main(void)
  {
      PIN_Handle ledPinHandle;
      Task_Params taskParams;

      // Call board init functions
      Board_initGeneral();
/*      Board_initSPI();
      Board_initI2C();
      Board_initUART();*/


      // Construct BIOS objects
      Task_Params_init(&taskParams);
      taskParams.stackSize = TASKSTACKSIZE;
      taskParams.stack = &task0Stack;
      Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);


      // Open LED pins
      ledPinHandle = PIN_open(&ledPinState, ledPinTable);

      if(!ledPinHandle) {
    	  System_printf("\nError initializing board LED pins\n");
    	  System_flush();
      }
      else{
    	  System_printf("\nPerfect! initializing board LED pins\n");
    	  System_flush();
      }

     // PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
      PIN_setOutputValue(ledPinHandle, Board_LED1, 1);

     // PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
    //  PIN_setOutputValue(ledPinHandle, Board_CP2103UARTRESET, 1);




      // Start BIOS
      BIOS_start();

      return (0);
  }
