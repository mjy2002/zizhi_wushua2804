#include "uart_app.h"
#include <string.h>

// ������ص�ִ�кͳ�ʼ��?

// ======================= ����ʹ�ܿ��� =======================
// ��ǰ�㻹û���� CubeMX ������ USART2 / USART3�������ȹر�
#define UART2_ENABLE    0
#define UART3_ENABLE    0
// ===========================================================



uint8_t uart1_rx_buff[512];

#if UART2_ENABLE
uint8_t uart2_rx_buff[512];
#endif

#if UART3_ENABLE
uint8_t uart3_rx_buff[512];
#endif


typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t *rx_buffer;
    uint16_t rx_len;              // ���ν��յ������ݳ���
    uint16_t buffer_size;
    volatile uint8_t rx_cplt_flag; // ������ɱ��?
} uart_device_t;


/**
 * @brief ���������ݽ�������
 * @param rx_buf ���ջ�����
 * @param len ���ݳ���
 * @param data ����������ݽṹ��?
 * @return 1:�����ɹ� 0:����ʧ��
 */



// ======================= UART1 �豸ʵ�� =======================
uart_device_t uart1_dev = {
    .huart = &huart1,
    .rx_buffer = uart1_rx_buff,
    .buffer_size = sizeof(uart1_rx_buff),
    .rx_len = 0,
    .rx_cplt_flag = 0
};


// ======================= UART2 �豸ʵ������ʱ�ر� =======================
#if UART2_ENABLE
uart_device_t uart2_dev = {
    .huart = &huart2,
    .rx_buffer = uart2_rx_buff,
    .buffer_size = sizeof(uart2_rx_buff),
    .rx_len = 0,
    .rx_cplt_flag = 0
};
#endif


// ======================= UART3 �豸ʵ������ʱ�ر� =======================
#if UART3_ENABLE
uart_device_t uart3_dev = {
    .huart = &huart3,
    .rx_buffer = uart3_rx_buff,
    .buffer_size = sizeof(uart3_rx_buff),
    .rx_len = 0,
    .rx_cplt_flag = 0
};
#endif


void uart_app_init(void)
{
    memset(uart1_dev.rx_buffer, 0, uart1_dev.buffer_size);

    HAL_UARTEx_ReceiveToIdle_DMA(
        uart1_dev.huart,
        uart1_dev.rx_buffer,
        uart1_dev.buffer_size
    );

    __HAL_DMA_DISABLE_IT(uart1_dev.huart->hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uart_device_t *dev = NULL;

    if (huart->Instance == USART1) {
        dev = &uart1_dev;
    }

#if UART2_ENABLE
    else if (huart->Instance == USART2) {
        dev = &uart2_dev;
    }
#endif

#if UART3_ENABLE
    else if (huart->Instance == USART3) {
        dev = &uart3_dev;
    }
#endif

    if (dev != NULL) {
        // Size �������Ǳ��ν��յ������ݳ���
        dev->rx_len = Size;

        // ���ñ�־λ��֪ͨ��ѭ������
        dev->rx_cplt_flag = 1;

        /*
         * ע�⣺
         * DMA Normal ģʽ�£�ReceiveToIdle_DMA ����һ�κ��ֹͣ��?
         * ���ﲻ�������� DMA�����ǵ���ѭ�����������ݺ���������
         * �������Ա������ݻ�û�����ͱ���һ�����ǡ�
         */
    }
}


// ͳһ�����ݴ�������
void uart_proc(void)
{
    // ======================= ���� USART1 =======================
    if (uart1_dev.rx_cplt_flag == 1) {

        /*
         * ����д UART1 �����ݴ����߼�
         * ���磺
         * my_printf(&huart1, "UART1 Recv: %d bytes\r\n", uart1_dev.rx_len);
         */
          // ��ֹ�ַ�����ӡԽ��
        if (uart1_dev.rx_len < uart1_dev.buffer_size) 
        {
            uart1_dev.rx_buffer[uart1_dev.rx_len] = '\0';
        }
        else 
        {
            uart1_dev.rx_buffer[uart1_dev.buffer_size - 1] = '\0';
        }


        // 2. 解析 "P,I,D,target" 格式，更�? PID 参数和目标�?
        PID_Tune_Parse((char *)uart1_dev.rx_buffer);
        
        // �����־�?
        uart1_dev.rx_cplt_flag = 0;
        uart1_dev.rx_len = 0;

        // ���¿��� DMA ������һ������
        HAL_UARTEx_ReceiveToIdle_DMA(
            uart1_dev.huart,
            uart1_dev.rx_buffer,
            uart1_dev.buffer_size
        );
    }


#if UART2_ENABLE
    // ======================= ���� USART2�����������ݽ��� =======================
    if (uart2_dev.rx_cplt_flag == 1) {

        // ��������������
        if (Parse_Sensor_Data(uart2_dev.rx_buffer, uart2_dev.rx_len, &g_sensor_data)) {
            // �����ɹ��������������ӡ����ʹ�ô���������?
            /*
            my_printf(&huart1,
                      "Temp: %.2f, Humi: %.2f, ADC1: %d, ADC2: %d\r\n",
                      g_sensor_data.temperature,
                      g_sensor_data.humidity,
                      g_sensor_data.adc1,
                      g_sensor_data.adc2);
            */
        } else {
            my_printf(&huart1, "UART2: Sensor data parse failed!\r\n");
        }

        // ����������
        uart2_dev.rx_cplt_flag = 0;
        uart2_dev.rx_len = 0;

        HAL_UARTEx_ReceiveToIdle_DMA(
            uart2_dev.huart,
            uart2_dev.rx_buffer,
            uart2_dev.buffer_size
        );
    }
#endif


#if UART3_ENABLE
    // ======================= ���� USART3 =======================
    if (uart3_dev.rx_cplt_flag == 1) {

        if (uart3_dev.rx_len < uart3_dev.buffer_size) {
            uart3_dev.rx_buffer[uart3_dev.rx_len] = '\0';
        } else {
            uart3_dev.rx_buffer[uart3_dev.buffer_size - 1] = '\0';
        }

        // ����д UART3 �����ݴ����߼�

        uart3_dev.rx_cplt_flag = 0;
        uart3_dev.rx_len = 0;

        HAL_UARTEx_ReceiveToIdle_DMA(
            uart3_dev.huart,
            uart3_dev.rx_buffer,
            uart3_dev.buffer_size
        );
    }
#endif
}

