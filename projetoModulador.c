#include "F28x_Project.h"
#include "math.h"
#include "driverlib.h"
#include "device.h"

__interrupt void LSPWM_isr(void); // Interrupção da modulação
void ConfigureTimer(void); // Configura a interrupção de tempo (atraves da qual o PWM é atualizado)
/*Necessário configurar 4 PWMs. São 8 chaves (1 fase) e cada PWM gera o comando para 2 chaves*/
void ConfigureEPWM(void);
void ConfigureDAC(void);
void initSCIAFIFO(void);

// Todos os PWMs têm o mesmo período
Uint16 PeriodoPWM  = 37500;

void main(void){

    InitSysCtrl();
    InitGpio();

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();

    ConfigureEPWM(); // Posso configurar todos os PWM em uma única função.
    ConfigureTimer();
    ConfigureDAC();

    DINT;

    InitPieCtrl();
    IER = 0;
    IFR = 0;
    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER0_INT = &LSPWM_isr;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; // Define pino 31 como output (LED Azul). Posso usá-lo como debug da interrupção de timer.
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;
    EDIS;


    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Configuration for the SCI Tx pin.
    //
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_SCITXDB );
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);

    EALLOW;
    //
    // Configuration for the SCI Rx pin.
    //
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_SCIRXDB);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);

    //
    // Initialize SCIB and its FIFO.
    //
    SCI_performSoftwareReset(SCIB_BASE);

    //
    // Configure SCIB for echoback.
    //
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, 9600, (SCI_CONFIG_WLEN_8 |
            SCI_CONFIG_STOP_ONE |
            SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);
    SCI_resetTxFIFO(SCIB_BASE);
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_disableLoopback(SCIB_BASE);
    SCI_enableFIFO(SCIB_BASE);
    SCI_enableModule(SCIB_BASE);
    SCI_performSoftwareReset(SCIB_BASE);

#ifdef AUTOBAUD
    //
    // Perform an autobaud lock.
    // SCI expects an 'a' or 'A' to lock the baud rate.
    //
    SCI_lockAutobaud(SCIB_BASE);
#endif
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Entender o que que essa linha faz.

    IER |= M_INT1;

    EINT;
    ERTM;

    while(1){

    }
}

void ConfigureTimer(void)
{
    CpuTimer0Regs.PRD.all = 3750; //A quantidade de contagens do PWM.

    /*
     * Explicando o PRD:
     * Quero uma frequência de 12kHz. 180e6/12e3 = 15e3;
     * Como está no modo up and down, esse valor deve ser a metade.
     * */
    CpuTimer0Regs.TCR.bit.TSS = 1;  // 1 = Stop timer, 0 = Start/Restart Timer
    CpuTimer0Regs.TCR.bit.TRB = 1;  // 1 = reload timer
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0; // Timer Free Run Disabled
    CpuTimer0Regs.TCR.bit.TIE = 1;  // 0 = Disable/ 1 = Enable Timer Interrupt
    CpuTimer0Regs.TCR.bit.TSS = 0;  // 1 = Stop timer, 0 = Start/Restart Timer
}

void ConfigureEPWM(void){
    EALLOW;
    EPwm1Regs.TBPRD = PeriodoPWM; // Determina o período da base de tempo do contador. Isso que define a frequência do PWM
    EPwm2Regs.TBPRD = PeriodoPWM; // TBPRD é o número de contagens feitas até o pwm resetar a contagem. 1 Contagem é feita por intervalo de tempo.
    EPwm3Regs.TBPRD = PeriodoPWM;
    EPwm4Regs.TBPRD = PeriodoPWM;

    // Time Base Control Register -> Define o modo
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;    // Counter Mode -> 2 = Up-Down
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;    // Counter Mode -> 2 = Up-Down
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;    // Counter Mode -> 2 = Up-Down
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;    // Counter Mode -> 2 = Up-Down
    EDIS;

    //Action Qualifier Control Register For Output A
    /*
     * É aqui que definimos oq eu vai acontecer quando o comparador for igual *
     * */
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Quando = 1 seta
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET; // Quando =1 reseta
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET; // Quando = 1 seta
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Quando =1 reseta

    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Quando = 1 seta
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET; // Quando =1 reseta
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET; // Quando = 1 seta
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Quando =1 reseta

    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Quando = 1 seta
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET; // Quando =1 reseta
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET; // Quando = 1 seta
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Quando =1 reseta

    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Quando = 1 seta
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET; // Quando =1 reseta
    EPwm4Regs.AQCTLB.bit.CBU = AQ_SET; // Quando = 1 seta
    EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR; // Quando =1 reseta

    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //    EPwm1Regs.DBCTL.bit.OUTSWAP = 0x00;
    //
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //    EPwm2Regs.DBCTL.bit.OUTSWAP = 0x00;
    //
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //    EPwm3Regs.DBCTL.bit.OUTSWAP = 0x00;
    //
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    //
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    //    EPwm4Regs.DBCTL.bit.OUTSWAP = 0x00;
    //
    EPwm1Regs.DBRED.bit.DBRED=1800;
    EPwm1Regs.DBFED.bit.DBFED=1800;

    EPwm2Regs.DBRED.bit.DBRED=1800;
    EPwm2Regs.DBFED.bit.DBFED=1800;

    EPwm3Regs.DBRED.bit.DBRED=1800;
    EPwm3Regs.DBFED.bit.DBFED=1800;

    EPwm4Regs.DBRED.bit.DBRED=1800;
    EPwm4Regs.DBFED.bit.DBFED=1800;
}

__interrupt void LSPWM_isr(void){
    static float tempo = 0, delt = 0.00002083333333333333;
    static float periodo = 0.016666666666667, seno = 0.00, invSeno = 0.00;
    static int InterruptCount = 0, resultsIndex = 0;
    InterruptCount++;

    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Verificar  GPIO31 para ver a frequência da interrupção
    GpioDataRegs.GPCTOGGLE.bit.GPIO67 = 1;

    seno = 0.8*sin(2*3.141592*60*tempo);
    invSeno = 0-seno;
    DacaRegs.DACVALS.all = (float)(2048 + 2000*seno);

    if (seno > 0){
        EPwm1Regs.CMPA.bit.CMPA = seno*PeriodoPWM;
        EPwm1Regs.CMPB.bit.CMPB = seno*PeriodoPWM;

        EPwm2Regs.CMPA.bit.CMPA = PeriodoPWM;
        EPwm2Regs.CMPB.bit.CMPB = PeriodoPWM;
    }else{
        EPwm1Regs.CMPA.bit.CMPA = 0;
        EPwm1Regs.CMPB.bit.CMPB = 0;

        EPwm2Regs.CMPA.bit.CMPA = seno*PeriodoPWM + PeriodoPWM;
        EPwm2Regs.CMPB.bit.CMPB = seno*PeriodoPWM + PeriodoPWM;
    }

    if (invSeno > 0){
        EPwm3Regs.CMPA.bit.CMPA = invSeno*PeriodoPWM;
        EPwm3Regs.CMPB.bit.CMPB = invSeno*PeriodoPWM;

        EPwm4Regs.CMPA.bit.CMPA = PeriodoPWM;
        EPwm4Regs.CMPB.bit.CMPB = PeriodoPWM;
    }else{
        EPwm3Regs.CMPA.bit.CMPA = 0;
        EPwm3Regs.CMPB.bit.CMPB = 0;

        EPwm4Regs.CMPA.bit.CMPA = invSeno*PeriodoPWM + PeriodoPWM;
        EPwm4Regs.CMPB.bit.CMPB = invSeno*PeriodoPWM + PeriodoPWM;
    }

    tempo += delt;
    if (tempo > periodo){
        tempo = tempo - periodo;
    }
    Uint16 voltage = 100*seno + 100;
    Uint16 frequency = 60;
    Uint16 mi = 8;
    Uint16 data[3];
    data[0] = voltage;
    data[1] = frequency;
    data[2] = mi;
    if(GpioDataRegs.GPBDAT.bit.GPIO59 == 1) //Pino 14
    {
        SCI_writeCharArray(SCIB_BASE, data, 3);
    }


    //    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void ConfigureDAC(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;       //use DAC references
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;      //enable DAC

    DacbRegs.DACCTL.bit.DACREFSEL = 1;       //use DAC references
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;      //enable DAC
    EDIS;
}




