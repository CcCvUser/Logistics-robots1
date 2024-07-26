#include "run_kws_model_demo.h"

#define KWS_LPUART                      LPUART8 
#define LPUART_RX_DMA_CHANNEL           1U                          //UART����ʹ�õ�DMAͨ����
#define LPUART_RX_DMA_REQUEST           kDmaRequestMuxLPUART8Rx     //���崮��DMA��������Դ
#define LPUART_DMAMUX_BASEADDR          DMAMUX                      //������ʹ�õ�DMA��·����ģ��(DMAMUX)
#define LPUART_DMA_BASEADDR             DMA0                        //����ʹ�õ�DMA
#define KWS_DATA_LENGTH                 16000                       //UART���պͷ������ݻ���������

lpuart_edma_handle_t g_lpuartEdmaHandle;  //����DMA������
edma_handle_t g_lpuartTxEdmaHandle;       //����DMA���;��
edma_handle_t g_lpuartRxEdmaHandle;       //����DMA���վ��
lpuart_transfer_t g_sendXfer;             //���巢�ʹ���ṹ��
lpuart_transfer_t g_receiveXfer;          //������մ���ṹ��
uint32_t g_bufflength = 0;

/* �շ������� */
AT_SDRAM_SECTION_ALIGN(int16_t audio_data[KWS_DATA_LENGTH], 4);

uint32_t get_data_lenth = 0;
uint8 audio_data_get_finish = 0;

tensor_dims_t inputDims;
tensor_type_t inputType;
tensor_dims_t outputDims;
tensor_type_t outputType;
uint8_t* inputData, *outputData;


//-------------------------------------------------------------------------------------------------------------------
// �������     ����DMA��ʼ��
// ����˵��     void
// ʹ��ʾ��     uart_dma_init();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
void uart_dma_init(void)
{
  edma_config_t config;
  
  DMAMUX_Init(LPUART_DMAMUX_BASEADDR);
  DMAMUX_SetSource(LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL, LPUART_RX_DMA_REQUEST);
  DMAMUX_EnableChannel(LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL);
  
  EDMA_GetDefaultConfig(&config);
  EDMA_Init(LPUART_DMA_BASEADDR, &config);
  EDMA_CreateHandle(&g_lpuartRxEdmaHandle, LPUART_DMA_BASEADDR, LPUART_RX_DMA_CHANNEL);
  
  LPUART_TransferCreateHandleEDMA(KWS_LPUART, &g_lpuartEdmaHandle, NULL, NULL, &g_lpuartTxEdmaHandle, &g_lpuartRxEdmaHandle);
  
  g_receiveXfer.data = (uint8*)audio_data;
  g_receiveXfer.dataSize = KWS_DATA_LENGTH * 2 + 1;
  LPUART_ReceiveEDMA(KWS_LPUART, &g_lpuartEdmaHandle, &g_receiveXfer);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����DMA�ص����������ڽ�������
// ����˵��     void
// ʹ��ʾ��     uart_dma_callback();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
void uart_dma_callback(void)
{
    LPUART_ClearStatusFlags(KWS_LPUART, kLPUART_IdleLineFlag);
    DCACHE_CleanInvalidateByRange((uint32_t)audio_data, KWS_DATA_LENGTH * 2);  
    LPUART_TransferGetReceiveCountEDMA(KWS_LPUART, &g_lpuartEdmaHandle, (uint32_t*)&g_bufflength);
    LPUART_TransferAbortReceiveEDMA(KWS_LPUART, &g_lpuartEdmaHandle);

    //��¼ʵ�����ݳ���
    get_data_lenth += (g_bufflength) / 2;
    
    //�ж�֡ͷ
    if( get_data_lenth >= 4 &&
        (uint16)audio_data[get_data_lenth - 4] == 0xa5a5    &&
        (uint16)audio_data[get_data_lenth - 3] == 0         &&
        (uint16)audio_data[get_data_lenth - 2] == 0         &&
        (uint16)audio_data[get_data_lenth - 1] == 0x007d )
    {
        get_data_lenth = 0;
        memset(audio_data, 0, sizeof(audio_data));
    }
    
    //������ݽ��������audio_data_get_finish��1
    if(get_data_lenth == KWS_DATA_LENGTH)
    {
        audio_data_get_finish = 1;
    }
    
    //���get_data_lenth�������ֵ�򽫳��ȵ���0
    if(get_data_lenth > KWS_DATA_LENGTH)get_data_lenth = 0;
    
    //��������DMA����
    g_receiveXfer.data = (uint8*)audio_data + get_data_lenth * 2;
    g_receiveXfer.dataSize = KWS_DATA_LENGTH * 2 + 1;
    LPUART_ReceiveEDMA(KWS_LPUART, &g_lpuartEdmaHandle, &g_receiveXfer);
    
}

//-------------------------------------------------------------------------------------------------------------------
// �������     tfliteģ�ͳ�ʼ��
// ����˵��     void
// ʹ��ʾ��     tflite_model_init();
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void tflite_model_init(void)
{
    //��ʼ��model
    if (MODEL_Init() != kStatus_Success)
    {
        //˵��ģ���ļ��쳣
        PRINTF("Failed initializing model" EOL);
        while(1);
    }
    
    inputData = MODEL_GetInputTensorData(&inputDims, &inputType);
    outputData = MODEL_GetOutputTensorData(&outputDims, &outputType);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��wifiת����
// ����˵��     void
// ʹ��ʾ��     audio_wifi_init();
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void audio_wifi_init(void)
{
	interrupt_global_disable();
	gpio_init(WIFI_UART_RTS_PIN, GPI, 0, GPI_PULL_UP);                          // ��ʼ����������
	gpio_init(WIFI_UART_RST_PIN, GPO, 1, GPO_PUSH_PULL);                        // ��ʼ����λ����
	uart_init(WIFI_UART_INDEX, 1500000, WIFI_UART_RX_PIN, WIFI_UART_TX_PIN);    // ��ʼ��WiFiģ����ʹ�õĴ���
    //��ʼ��wifiת���ڽ���
    uart_dma_init();
    uart_rx_idle_interrupt (WIFI_UART_INDEX, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ʶ��
// ����˵��     void
// ���ز���     int           ������ǩֵ
// ʹ��ʾ��     audio_predict();
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
int audio_predict(void)
{
    int label_index;
    //��wifi���յ����ݴ�����������
    set_audio_data((int16_t*)&audio_data[0]);
    
    //����Ԥ����
    if (AUDIO_GetSpectralSample(inputData, inputDims.data[1] * inputDims.data[2]) != kStatus_Success)
    {
        PRINTF("Failed retrieving input audio" EOL);
        while(1);
    }
    //ģ������
    MODEL_RunInference();
    
    //��¼��ɵ�ֵ����ʼ��ֵ��dma�ص������м�¼

    //ģ��ʶ�����
    MODEL_ProcessOutput(outputData, &outputDims, outputType, 0, &label_index);
    
    //����������ǩֵ
    return label_index;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ʹ�ù̶���������ʶ��
// ����˵��     void
// ���ز���     int           ������ǩֵ
// ʹ��ʾ��     audio_predict();
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
int audio_predict_use_data(void)
{
    int label_index;
    //���̶����ݴ�����������
    set_audio_data((int16_t*)self_sample_data);
    
    //����Ԥ����
    if (AUDIO_GetSpectralSample(inputData, inputDims.data[1] * inputDims.data[2]) != kStatus_Success)
    {
        PRINTF("Failed retrieving input audio" EOL);
        while(1);
    }
    //ģ������
    MODEL_RunInference();
    
    //��¼��ɵ�ֵ����ʼ��ֵ��dma�ص������м�¼

    //ģ��ʶ�����
    MODEL_ProcessOutput(outputData, &outputDims, outputType, 0, &label_index);
    
    //����������ǩֵ
    return label_index;
}