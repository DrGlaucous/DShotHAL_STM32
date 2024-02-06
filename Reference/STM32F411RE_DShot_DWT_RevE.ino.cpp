#include "stm32yyxx_ll.h"

#define USE_DSHOT
#define USE_DSHOT_DMAR
#define MAX_SUPPORTED_MOTORS 8

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       19

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
uint32_t dmaBufferT3CH1[DSHOT_DMA_BUFFER_SIZE];
uint32_t dmaBufferT3CH2[DSHOT_DMA_BUFFER_SIZE];
uint32_t dmaBufferT4CH1[DSHOT_DMA_BUFFER_SIZE];
uint32_t dmaBufferT4CH2[DSHOT_DMA_BUFFER_SIZE];

// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
typedef uint8_t ioTag_t;       // packet tag to specify IO pin
typedef void* IO_t;            // type specifying IO pin. Currently ioRec_t pointer, but this may change
typedef uint8_t ioConfig_t;  // packed IO configuration

// DEF_TIM(TIM3, CH1, PB4,  TIM_USE_MOTOR,  0, 0), // S1_OUT
// DEF_TIM(TIM3, CH2, PB5,  TIM_USE_MOTOR,  0, 0), // S2_OUT
// DEF_TIM(TIM4, CH1, PB6,  TIM_USE_MOTOR,  0, 0), // S3_OUT
// DEF_TIM(TIM4, CH2, PB7,  TIM_USE_MOTOR,  0, 0), // S4_OUT

int cnt = 0;

int32_t Timer1;
int32_t Timer2;
int32_t Timer3;
int32_t Timer4;

#define testLoop1_Pin PB6
#define testLoop1_Port GPIOB
#define testLoop1_PinNum LL_GPIO_PIN_6

#define testLoop2_Pin PB7
#define testLoop2_Port GPIOB
#define testLoop2_PinNum LL_GPIO_PIN_7


// STM32F411 Reference Manual RM0383 pp.170
extern "C" void DMA1_Stream0_IRQHandler(void) {

  //LL_GPIO_SetOutputPin(testLoop1_Port, testLoop1_PinNum);

  if (LL_DMA_IsActiveFlag_TC0(DMA1)) {

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    LL_TIM_DisableDMAReq_CC1(TIM4);

    LL_DMA_ClearFlag_TC0(DMA1);

  }

  //LL_GPIO_ResetOutputPin(testLoop1_Port, testLoop1_PinNum);

}

extern "C" void DMA1_Stream3_IRQHandler(void) {

  //LL_GPIO_SetOutputPin(testLoop1_Port, testLoop1_PinNum);

  if (LL_DMA_IsActiveFlag_TC3(DMA1)) {

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
    LL_TIM_DisableDMAReq_CC2(TIM4);

    LL_DMA_ClearFlag_TC3(DMA1);

  }

  //LL_GPIO_ResetOutputPin(testLoop1_Port, testLoop1_PinNum);

}

extern "C" void DMA1_Stream4_IRQHandler(void) {

  //LL_GPIO_SetOutputPin(testLoop1_Port, testLoop1_PinNum);

  if (LL_DMA_IsActiveFlag_TC4(DMA1)) {

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
    LL_TIM_DisableDMAReq_CC1(TIM3);

    LL_DMA_ClearFlag_TC4(DMA1);

  }

  //LL_GPIO_ResetOutputPin(testLoop1_Port, testLoop1_PinNum);

}


extern "C" void DMA1_Stream5_IRQHandler(void) {

  //LL_GPIO_SetOutputPin(testLoop2_Port, testLoop2_PinNum);

  if (LL_DMA_IsActiveFlag_TC5(DMA1)) {

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
    LL_TIM_DisableDMAReq_CC2(TIM3);

    LL_DMA_ClearFlag_TC5(DMA1);

  }

  //LL_GPIO_ResetOutputPin(testLoop2_Port, testLoop2_PinNum);

}


void setup() {

  Serial.begin(115200);
  Serial.print("Initializing...");
  Serial.print(HAL_RCC_GetHCLKFreq());
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  DWT_Delay_Init();
  
  pwmDshotMotorHardwareConfig();

}


void loop() {

  uint16_t packet1, packet2, packet3, packet4;
  uint8_t bufferSize1, bufferSize2, bufferSize3, bufferSize4;

  packet1 = prepareDshotPacket( (cnt & 0b11111111111) , false);
  bufferSize1 = loadDmaBufferDshot((uint32_t *)dmaBufferT3CH1, 1, packet1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, bufferSize1);

  packet2 = prepareDshotPacket( ((cnt+1) & 0b11111111111) , false);
  bufferSize2 = loadDmaBufferDshot((uint32_t *)dmaBufferT3CH2, 1, packet2);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, bufferSize2);

  packet3 = prepareDshotPacket( ((cnt+2) & 0b11111111111) , false);
  bufferSize3 = loadDmaBufferDshot((uint32_t *)dmaBufferT4CH1, 1, packet3);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, bufferSize3);

  packet4 = prepareDshotPacket( ((cnt+3) & 0b11111111111) , false);
  bufferSize4 = loadDmaBufferDshot((uint32_t *)dmaBufferT4CH2, 1, packet4);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, bufferSize4);
  
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);

  //TIM_SetCounter(dmaMotorTimers[i].timer, 0);
  LL_TIM_SetCounter(TIM3, 0);
  LL_TIM_SetCounter(TIM4, 0);
  
  //TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
  LL_TIM_EnableDMAReq_CC1(TIM3);
  LL_TIM_EnableDMAReq_CC2(TIM3);
  LL_TIM_EnableDMAReq_CC1(TIM4);
  LL_TIM_EnableDMAReq_CC2(TIM4);
  
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on (HIGH is the voltage level)
  DWT_Delay_us(12);                   // wait for fraction of a second
  digitalWrite(LED_BUILTIN, LOW);     // turn the LED off by making the voltage LOW
  DWT_Delay_us(12);                   // wait for fraction of a second

  cnt++;
  if (cnt > 2047) {
    cnt = 0;
  }

}


#define MHZ_TO_HZ(x) ((x) * 1000000)
#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

void pwmDshotMotorHardwareConfig(void) {

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO peripheral clock enable
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  // TIM3 GPIO Configuration  
// DEF_TIM(TIM3, CH1, PB4,  TIM_USE_MOTOR,  0, 0), // S1_OUT
// DEF_TIM(TIM3, CH2, PB5,  TIM_USE_MOTOR,  0, 0), // S2_OUT
// DEF_TIM(TIM4, CH1, PB6,  TIM_USE_MOTOR,  0, 0), // S3_OUT
// DEF_TIM(TIM4, CH2, PB7,  TIM_USE_MOTOR,  0, 0), // S4_OUT
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // TIM3 peripheral clock enable
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  LL_TIM_DisableCounter(TIM3);
  // TIM4 peripheral clock enable
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  LL_TIM_DisableCounter(TIM4);

  LL_TIM_InitTypeDef TIM_TimeBaseStructure;
  LL_TIM_StructInit(&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.Prescaler = (uint16_t)(((float) 100000000 / MOTOR_DSHOT600_HZ + 0.01f) - 1);
  TIM_TimeBaseStructure.Autoreload = MOTOR_BITLENGTH;
  TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.RepetitionCounter = 0;
  TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;

  LL_TIM_Init(TIM3, &TIM_TimeBaseStructure);
  LL_TIM_Init(TIM4, &TIM_TimeBaseStructure);

  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
  
  LL_TIM_OC_StructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OCInitStructure.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  TIM_OCInitStructure.OCPolarity =  LL_TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.CompareValue = 0;

  // Init TIM3 and TIM4 output compare channels
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OCInitStructure);
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OCInitStructure);

  // TIM3 and TIM4 OC enable preload
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);

  // TIM3 and TIM4 OC enable channel 
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);

  LL_TIM_EnableAllOutputs(TIM3);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_EnableCounter(TIM3);

  LL_TIM_EnableAllOutputs(TIM4);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_EnableCounter(TIM4);

  //---DMA-------------------------------------

  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
  
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_4);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_5);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_0);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_3);

  LL_DMA_InitTypeDef DMA_InitStructure;
  
  // DMA controller clock enable
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  //-DMA TIM3 CCR1-----------

  LL_DMA_StructInit(&DMA_InitStructure);

  DMA_InitStructure.Channel = LL_DMA_CHANNEL_5;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)dmaBufferT3CH1;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
  DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;

  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM3->CCR1;
  DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;

  LL_DMA_Init(DMA1, LL_DMA_STREAM_4, &DMA_InitStructure);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);

  //-DMA TIM3 CCR2-----------

  DMA_InitStructure.Channel = LL_DMA_CHANNEL_5;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)dmaBufferT3CH2;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM3->CCR2;

  LL_DMA_Init(DMA1, LL_DMA_STREAM_5, &DMA_InitStructure);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);

  //-DMA TIM4 CCR1-----------
  
  DMA_InitStructure.Channel = LL_DMA_CHANNEL_2;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)dmaBufferT4CH1;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM4->CCR1;

  LL_DMA_Init(DMA1, LL_DMA_STREAM_0, &DMA_InitStructure);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

  //-DMA TIM4 CCR2-----------

  DMA_InitStructure.Channel = LL_DMA_CHANNEL_2;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)dmaBufferT4CH2;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM4->CCR2;

  LL_DMA_Init(DMA1, LL_DMA_STREAM_3, &DMA_InitStructure);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);

}


uint16_t prepareDshotPacket(uint16_t value, bool requestTelemetry) {

  uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
  requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row

  // compute checksum
  int csum = 0;
  int csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^=  csum_data;   // xor data by nibbles
    csum_data >>= 4;
  }
  csum &= 0xf;
  // append checksum
  packet = (packet << 4) | csum;

  return packet;

}


static uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet) {

  for (int i = 0; i < 16; i++) {
    dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
    packet <<= 1;
  }

  return DSHOT_DMA_BUFFER_SIZE;

}
