/*****************************************************************************
* @FileName: sfud.c
* @Engineer: armink & TenYan
* @Company:  徐工信息智能硬件部
* @Date:     2020-7-6
* @brief     This file is part of the Serial Flash Universal Driver Library.
******************************************************************************/
#include "sfud.h"
#include <string.h>
#include "stm32f2xx_conf.h"
#include "main.h"
#include "cmsis_os2.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
#define DUMMY_DATA     0xFF  // send dummy data for read data

#define SFUD_SPI                   SPI2
#define SFUD_SPI_CLK               RCC_APB1Periph_SPI2
#define SFUD_SPI_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define SFUD_SPI_GPIO_PORT         GPIOB

#define SFUD_SPI_SCK_PIN           GPIO_Pin_10              // PB.10
#define SFUD_SPI_SCK_GPIO_PORT     GPIOB                   // GPIOB
#define SFUD_SPI_SCK_GPIO_CLK      RCC_AHB1Periph_GPIOB

#define SFUD_SPI_MISO_PIN          GPIO_Pin_2              // PC.2
#define SFUD_SPI_MISO_GPIO_PORT    GPIOC                   // GPIOC
#define SFUD_SPI_MISO_GPIO_CLK     RCC_AHB1Periph_GPIOC

#define SFUD_SPI_MOSI_PIN          GPIO_Pin_3             // PC.3
#define SFUD_SPI_MOSI_GPIO_PORT    GPIOC                  // GPIOC
#define SFUD_SPI_MOSI_GPIO_CLK     RCC_AHB1Periph_GPIOC

#define SFUD_CS_PIN                GPIO_Pin_9             // PB.9
#define SFUD_CS_GPIO_PORT          GPIOB                  // GPIOB
#define SFUD_CS_GPIO_CLK           RCC_AHB1Periph_GPIOB

#define sfud_select()              SFUD_CS_GPIO_PORT->BSRRH = SFUD_CS_PIN  // SPIFLASH_CS_L()
#define sfud_deselect()            SFUD_CS_GPIO_PORT->BSRRL = SFUD_CS_PIN  // SPIFLASH_CS_H()


/******************************************************************************
* Data Types and Globals
******************************************************************************/
enum
{
  SFUD_MX25L3206E_DEVICE_INDEX = 0,
};

// | name | mf_id | type_id | capacity_id | capacity | write_mode | erase_gran | erase_gran_cmd |
sfud_flash_t sfud_mx25l3206e = {
	.index = SFUD_MX25L3206E_DEVICE_INDEX,
  .name = "MX25L3206E",
  .spi.name = "SPI2",
  .chip = { "MX25L3206E", SFUD_MF_ID_MICRONIX, 0x20, 0x16, 4L * 1024L * 1024L, SFUD_WM_PAGE_256B, 4096, 0x20 } 
};

typedef struct 
{
  SPI_TypeDef *spix;
  GPIO_TypeDef *cs_gpiox;
  uint16_t cs_gpio_pin;
} spi_user_data, *spi_user_data_t;
static spi_user_data spi1 = { .spix = SPI2, .cs_gpiox = GPIOB, .cs_gpio_pin = GPIO_Pin_9 };

osSemaphoreId_t sid_spi2_semaphore = NULL;

/******************************************************************************
* function prototypes
******************************************************************************/
static sfud_status_t page256_or_1_byte_write(const sfud_flash_t *flash, uint32_t addr, size_t size, uint16_t write_gran,const uint8_t *data);
static sfud_status_t aai_write(const sfud_flash_t *flash, uint32_t addr, size_t size, const uint8_t *data);
static sfud_status_t wait_busy(const sfud_flash_t *flash);
static sfud_status_t reset(const sfud_flash_t *flash);
static sfud_status_t set_write_enabled(const sfud_flash_t *flash, bool enabled);
static sfud_status_t set_4_byte_address_mode(sfud_flash_t *flash, bool enabled);
static void make_adress_byte_array(const sfud_flash_t *flash, uint32_t addr, uint8_t *array);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 硬件移植函数
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void rcc_configuration(spi_user_data_t spi)
{
  if (spi->spix == SPI1)
  {
    RCC_APB2PeriphClockCmd(SFUD_SPI_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(SFUD_SPI_GPIO_CLK, ENABLE);
  }
  else if (spi->spix == SPI2)
  {
    RCC_APB1PeriphClockCmd(SFUD_SPI_CLK, ENABLE);
    //RCC_APB1PeriphClockCmd(SFUD_SPI_GPIO_CLK,ENABLE);
    RCC_APB1PeriphClockCmd(SFUD_SPI_SCK_GPIO_CLK,ENABLE);
    RCC_APB1PeriphClockCmd(SFUD_SPI_MISO_GPIO_CLK,ENABLE);
    RCC_APB1PeriphClockCmd(SFUD_SPI_MOSI_GPIO_CLK,ENABLE);
    RCC_APB1PeriphClockCmd(SFUD_CS_GPIO_CLK,ENABLE);
  }
}

//====================================================================================
static void gpio_configuration(spi_user_data_t spi)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if (spi->spix == SPI1)
  {

  }
  else if (spi->spix == SPI2)
  {
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI1); // Connect PB10 to SPI2_SCK
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI1); // Connect PC2 to SPI2_MISO
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI1); // Connect PC3 to SPI2_MOSI

    // SCK:PB10 CS:PB9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    

    // MISO:PC2  MOSI:PC3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    sfud_deselect();
  }
}

//====================================================================================
static void spi_configuration(spi_user_data_t spi)
{
  SPI_InitTypeDef SPI_InitStructure;

  SPI_I2S_DeInit(spi->spix);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI 设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //设置为主 SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI 发送接收 8 位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                         //时钟悬空低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                       //数据捕获于第一个时钟沿
  //TODO 以后可以尝试硬件 CS
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          //内部  NSS 信号由 SSI 位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //波特率预分频值为 4
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 //数据传输从 MSB 位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;                           // CRC 值计算的多项式

  SPI_Init(spi->spix, &SPI_InitStructure);
  SPI_Cmd(spi->spix, ENABLE);

  if(sid_spi2_semaphore==NULL)
  {
    sid_spi2_semaphore = osSemaphoreNew(1, 1, NULL);;   //创建SPI1通信的互斥信号量
  }
}

//====================================================================================
static void spi_lock(const sfud_spi_t *spi)
{
  osSemaphoreAcquire(sid_spi2_semaphore, osWaitForever);;
}

static void spi_unlock(const sfud_spi_t *spi)
{
  osSemaphoreRelease(sid_spi2_semaphore);;
}

/*************************************************************************
 * SPI write data then read data
*************************************************************************/
static sfud_status_t spi_write_read(const sfud_spi_t *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf,size_t read_size)
{
  sfud_status_t result = SFUD_SUCCESS;
  uint8_t send_data, read_data;
  spi_user_data_t spi_dev = (spi_user_data_t) spi->user_data;

  GPIO_ResetBits(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin);
  // 开始读写数据
  for (size_t i = 0, retry_times; i < write_size + read_size; i++)
  {
    // 先写缓冲区中的数据到 SPI 总线，数据写完后，再写 dummy(0xFF) 到 SPI 总线
    if (i < write_size)
    {
      send_data = *write_buf++;
    }
    else
    {
      send_data = SFUD_DUMMY_DATA;
    }
    // 发送数据
    retry_times = 5000;
    while (SPI_I2S_GetFlagStatus(spi_dev->spix, SPI_I2S_FLAG_TXE) == RESET) {
      SFUD_RETRY_PROCESS(NULL, retry_times, result);
    }
    if (result != SFUD_SUCCESS) {
      goto exit;
    }
    SPI_I2S_SendData(spi_dev->spix, send_data);
    // 接收数据
    retry_times = 5000;
    while (SPI_I2S_GetFlagStatus(spi_dev->spix, SPI_I2S_FLAG_RXNE) == RESET)
    {
      SFUD_RETRY_PROCESS(NULL, retry_times, result);
    }
    if (result != SFUD_SUCCESS)
    {
      goto exit;
    }
    read_data = SPI_I2S_ReceiveData(spi_dev->spix);
    // 写缓冲区中的数据发完后，再读取 SPI 总线中的数据到读缓冲区
    if (i >= write_size)
    {
      *read_buf++ = read_data;
    }
  }

exit:
  GPIO_SetBits(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin);

  return result;
}

//====================================================================================
static void retry_delay_100us(void)
{
  uint32_t delay = 120;
  while (delay--);
}

//====================================================================================
sfud_status_t sfud_spi_port_init(sfud_flash_t *flash)
{
  sfud_status_t result = SFUD_SUCCESS;

  switch (flash->index)
  {
  case SFUD_MX25L3206E_DEVICE_INDEX:
    rcc_configuration(&spi1);  // RCC 初始化
    gpio_configuration(&spi1); // GPIO 初始化
    spi_configuration(&spi1);  // SPI 外设初始化

    // 同步 Flash 移植所需的接口及数据
    flash->spi.rw = spi_write_read;
    flash->spi.lock = spi_lock;
    flash->spi.unlock = spi_unlock;
    flash->spi.user_data = &spi1;

    flash->retry.delay = retry_delay_100us; // about 100 microsecond delay
    flash->retry.times = 60 * 10000; // adout 60 seconds timeout
    break;
  }

  return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 用户不可修改
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************************
 * hardware initialize
*************************************************************************/
static sfud_status_t hardware_init(sfud_flash_t *flash)
{
  sfud_status_t result = SFUD_SUCCESS;
  //size_t i;

  result = sfud_spi_port_init(flash);
  if (result != SFUD_SUCCESS)
  {
    return result;
  }

  /* reset flash device */
  result = reset(flash);
  if (result != SFUD_SUCCESS)
  {
    return result;
  }

  // I found when the flash write mode is supported AAI mode. The flash all blocks is protected,
  // so need change the flash status to unprotected before write and erase operate.
  if (flash->chip.write_mode & SFUD_WM_AAI)
  {
    result = sfud_write_status(flash, true, 0x00);
    if (result != SFUD_SUCCESS)
    {
      return result;
    }
  }

  // if the flash is large than 16MB (256Mb) then enter in 4-Byte addressing mode
  if (flash->chip.capacity > (1L << 24))
  {
    result = set_4_byte_address_mode(flash, true);
  }
  else
  {
    flash->addr_in_4_byte = false;
  }

  return result;
}

/*************************************************************************
 * SFUD initialize by flash device
 * @param flash flash device
 * @return result
*************************************************************************/
sfud_status_t sfud_device_init(sfud_flash_t *flash)
{
  sfud_status_t result = SFUD_SUCCESS;

  // hardware initialize
  result = hardware_init(flash);

  if (result == SFUD_SUCCESS)
  {
    flash->init_ok = true;
  }
  else
  {
    flash->init_ok = false;
  }

  return result;
}

/*************************************************************************
 * SFUD library initialize.
 * @return result
*************************************************************************/
sfud_status_t sfud_init(void)
{
  sfud_status_t cur_flash_result = SFUD_SUCCESS, all_flash_result = SFUD_SUCCESS;
  //size_t i;

  // initialize the flash device
  cur_flash_result = sfud_device_init(&sfud_mx25l3206e);
  if (cur_flash_result != SFUD_SUCCESS)
  {
    all_flash_result = cur_flash_result;
  }

  return all_flash_result;
}

/*************************************************************************
 * read flash data
 * @param flash flash device
 * @param addr start address
 * @param size read size
 * @param data read data pointer
 * @return result
*************************************************************************/
sfud_status_t sfud_read(const sfud_flash_t *flash, uint32_t addr, size_t size, uint8_t *data)
{
  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  uint8_t cmd_data[5], cmd_size;

  if (addr + size > flash->chip.capacity) // check the flash address bound
  {
    return SFUD_ERR_ADDR_OUT_OF_BOUND;
  }
  // lock SPI
  if (spi->lock)
  {
    spi->lock(spi);
  }

  result = wait_busy(flash);

  if (result == SFUD_SUCCESS)
  {
    cmd_data[0] = SFUD_CMD_READ_DATA;
    make_adress_byte_array(flash, addr, &cmd_data[1]);
    cmd_size = flash->addr_in_4_byte ? 5 : 4;
    result = spi->rw(spi, cmd_data, cmd_size, data, size);
  }
  // unlock SPI
  if (spi->unlock)
  {
    spi->unlock(spi);
  }

  return result;
}

/*************************************************************************
 * erase all flash data
 * @param flash flash device
 * @return result
*************************************************************************/
sfud_status_t sfud_chip_erase(const sfud_flash_t *flash)
{
  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  uint8_t cmd_data[4];

  // lock SPI
  if (spi->lock)
  {
    spi->lock(spi);
  }

  result = set_write_enabled(flash, true); // set the flash write enable
  if (result != SFUD_SUCCESS)
  {
    goto __exit;
  }

  cmd_data[0] = SFUD_CMD_ERASE_CHIP;
  // dual-buffer write, like AT45DB series flash chip erase operate is different for other flash
  if (flash->chip.write_mode & SFUD_WM_DUAL_BUFFER)
  {
    cmd_data[1] = 0x94;
    cmd_data[2] = 0x80;
    cmd_data[3] = 0x9A;
    result = spi->rw(spi, cmd_data, 4, NULL, 0);
  }
  else
  {
    result = spi->rw(spi, cmd_data, 1, NULL, 0);
  }
  if (result != SFUD_SUCCESS)
  {
    goto __exit;
  }
  result = wait_busy(flash);

__exit:
  // set the flash write disable
  set_write_enabled(flash, false);
  // unlock SPI
  if (spi->unlock)
  {
    spi->unlock(spi);
  }

  return result;
}

/*************************************************************************
 * erase flash data
 * @note It will erase align by erase granularity.
 * @param flash flash device
 * @param addr start address
 * @param size erase size
 * @return result
*************************************************************************/
sfud_status_t sfud_erase(const sfud_flash_t *flash, uint32_t addr, size_t size)
{
  extern size_t sfud_sfdp_get_suitable_eraser(const sfud_flash_t *flash, uint32_t addr, size_t erase_size);

  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  uint8_t cmd_data[5], cmd_size, cur_erase_cmd;
  size_t cur_erase_size;

  // check the flash address bound
  if (addr + size > flash->chip.capacity)
  {
    return SFUD_ERR_ADDR_OUT_OF_BOUND;
  }

  if (addr == 0 && size == flash->chip.capacity)
  {
    return sfud_chip_erase(flash);
  }

  // lock SPI
  if (spi->lock)
  {
    spi->lock(spi);
  }

  // loop erase operate. erase unit is erase granularity
  while (size)
  {
    ClrWdt();  // 清除看门狗
    //WDOG_LED_ON(); // 清除外部看门狗

    cur_erase_cmd = flash->chip.erase_gran_cmd;
    cur_erase_size = flash->chip.erase_gran;

    // set the flash write enable
    result = set_write_enabled(flash, true);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }

    cmd_data[0] = cur_erase_cmd;
    make_adress_byte_array(flash, addr, &cmd_data[1]);
    cmd_size = flash->addr_in_4_byte ? 5 : 4;
    result = spi->rw(spi, cmd_data, cmd_size, NULL, 0);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }
    result = wait_busy(flash);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }
    // make erase align and calculate next erase address
    if (addr % cur_erase_size != 0)
    {
      if (size > cur_erase_size - (addr % cur_erase_size))
      {
        size -= cur_erase_size - (addr % cur_erase_size);
        addr += cur_erase_size - (addr % cur_erase_size);
      }
      else
      {
        goto __exit;
      }
    }
    else
    {
      if (size > cur_erase_size)
      {
        size -= cur_erase_size;
        addr += cur_erase_size;
      }
      else
      {
        goto __exit;
      }
    }
    //WDOG_LED_OFF(); // 清除外部看门狗
  }

__exit:
  // set the flash write disable
  set_write_enabled(flash, false);
  // unlock SPI
  if (spi->unlock)
  {
    spi->unlock(spi);
  }

  return result;
}

/*************************************************************************
 * write flash data (no erase operate) for write 1 to 256 bytes per page mode or byte write mode
 * @param flash flash device
 * @param addr start address
 * @param size write size
 * @param write_gran write granularity bytes, only support 1 or 256
 * @param data write data
 * @return result
*************************************************************************/
static sfud_status_t page256_or_1_byte_write(const sfud_flash_t *flash, uint32_t addr, size_t size, uint16_t write_gran,const uint8_t *data)
{
  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  static uint8_t cmd_data[5 + SFUD_WRITE_MAX_PAGE_SIZE];
  uint8_t cmd_size;
  size_t data_size;

  // check the flash address bound
  if (addr + size > flash->chip.capacity)
  {
    return SFUD_ERR_ADDR_OUT_OF_BOUND;
  }
  // lock SPI
  if (spi->lock)
  {
    spi->lock(spi);
  }

  // loop write operate. write unit is write granularity
  while (size)
  {
    // set the flash write enable
    result = set_write_enabled(flash, true);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }
    cmd_data[0] = SFUD_CMD_PAGE_PROGRAM;
    make_adress_byte_array(flash, addr, &cmd_data[1]);
    cmd_size = flash->addr_in_4_byte ? 5 : 4;

    // make write align and calculate next write address
    if (addr % write_gran != 0)
    {
      if (size > write_gran - (addr % write_gran))
      {
        data_size = write_gran - (addr % write_gran);
      }
      else
      {
        data_size = size;
      }
    }
    else
    {
      if (size > write_gran)
      {
        data_size = write_gran;
      }
      else
      {
        data_size = size;
      }
    }
    size -= data_size;
    addr += data_size;

    memcpy(&cmd_data[cmd_size], data, data_size);

    result = spi->rw(spi, cmd_data, cmd_size + data_size, NULL, 0);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }
    result = wait_busy(flash);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }
    data += data_size;
  }

__exit:
  // set the flash write disable
  set_write_enabled(flash, false);
  // unlock SPI
  if (spi->unlock)
  {
    spi->unlock(spi);
  }

  return result;
}

/*************************************************************************
 * write flash data (no erase operate) for auto address increment mode
 * If the address is odd number, it will place one 0xFF before the start of data for protect the old data.
 * If the latest remain size is 1, it will append one 0xFF at the end of data for protect the old data.
 * @param flash flash device
 * @param addr start address
 * @param size write size
 * @param data write data
 * @return result
*************************************************************************/
static sfud_status_t aai_write(const sfud_flash_t *flash, uint32_t addr, size_t size, const uint8_t *data)
{
  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  uint8_t cmd_data[8], cmd_size;
  bool first_write = true;

  // check the flash address bound
  if (addr + size > flash->chip.capacity)
  {
    return SFUD_ERR_ADDR_OUT_OF_BOUND;
  }
  // lock SPI
  if (spi->lock)
  {
    spi->lock(spi);
  }
  // The address must be even for AAI write mode. So it must write one byte first when address is odd.
  if (addr % 2 != 0)
  {
    result = page256_or_1_byte_write(flash, addr++, 1, 1, data++);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }
    size--;
  }
  // set the flash write enable
  result = set_write_enabled(flash, true);
  if (result != SFUD_SUCCESS)
  {
    goto __exit;
  }
  // loop write operate.
  cmd_data[0] = SFUD_CMD_AAI_WORD_PROGRAM;
  while (size >= 2)
  {
    if (first_write)
    {
      make_adress_byte_array(flash, addr, &cmd_data[1]);
      cmd_size = flash->addr_in_4_byte ? 5 : 4;
      cmd_data[cmd_size] = *data;
      cmd_data[cmd_size + 1] = *(data + 1);
      first_write = false;
    }
    else
    {
      cmd_size = 1;
      cmd_data[1] = *data;
      cmd_data[2] = *(data + 1);
    }

    result = spi->rw(spi, cmd_data, cmd_size + 2, NULL, 0);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }

    result = wait_busy(flash);
    if (result != SFUD_SUCCESS)
    {
      goto __exit;
    }

    size -= 2;
    addr += 2;
    data += 2;
  }
  // set the flash write disable for exit AAI mode
  result = set_write_enabled(flash, false);
  // write last one byte data when origin write size is odd
  if (result == SFUD_SUCCESS && size == 1)
  {
    result = page256_or_1_byte_write(flash, addr, 1, 1, data);
  }

__exit:
  if (result != SFUD_SUCCESS)
  {
    set_write_enabled(flash, false);
  }
  // unlock SPI
  if (spi->unlock)
  {
    spi->unlock(spi);
  }

  return result;
}

/*************************************************************************
 * write flash data (no erase operate)
 * @param flash flash device
 * @param addr start address
 * @param size write size
 * @param data write data
 * @return result
*************************************************************************/
sfud_status_t sfud_write(const sfud_flash_t *flash, uint32_t addr, size_t size, const uint8_t *data)
{
  sfud_status_t result = SFUD_SUCCESS;

  if (flash->chip.write_mode & SFUD_WM_PAGE_256B)
  {
    result = page256_or_1_byte_write(flash, addr, size, 256, data);
  }
  else if (flash->chip.write_mode & SFUD_WM_AAI)
  {
    result = aai_write(flash, addr, size, data);
  }
  else if (flash->chip.write_mode & SFUD_WM_DUAL_BUFFER)
  {
    //TODO dual-buffer write mode
  }

  return result;
}

/*************************************************************************
 * erase and write flash data
 * @param flash flash device
 * @param addr start address
 * @param size write size
 * @param data write data
 * @return result
*************************************************************************/
sfud_status_t sfud_erase_write(const sfud_flash_t *flash, uint32_t addr, size_t size, const uint8_t *data)
{
  sfud_status_t result = SFUD_SUCCESS;

  result = sfud_erase(flash, addr, size);

  if (result == SFUD_SUCCESS)
  {
    result = sfud_write(flash, addr, size, data);
  }

  return result;
}

//====================================================================================
static sfud_status_t reset(const sfud_flash_t *flash)
{
  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  uint8_t cmd_data[2];

  cmd_data[0] = SFUD_CMD_ENABLE_RESET;
  result = spi->rw(spi, cmd_data, 1, NULL, 0);
  if (result == SFUD_SUCCESS)
  {
    result = wait_busy(flash);
  }
  else
  {
    return result;
  }

  cmd_data[1] = SFUD_CMD_RESET;
  result = spi->rw(spi, &cmd_data[1], 1, NULL, 0);

  if (result == SFUD_SUCCESS)
  {
    result = wait_busy(flash);
  }

  return result;
}

/*************************************************************************
 * set the flash write enable or write disable
 * @param flash flash device
 * @param enabled true: enable  false: disable
 * @return result
*************************************************************************/
static sfud_status_t set_write_enabled(const sfud_flash_t *flash, bool enabled)
{
  sfud_status_t result = SFUD_SUCCESS;
  uint8_t cmd, register_status;

  if (enabled)
  {
    cmd = SFUD_CMD_WRITE_ENABLE;
  }
  else
  {
    cmd = SFUD_CMD_WRITE_DISABLE;
  }

  result = flash->spi.rw(&flash->spi, &cmd, 1, NULL, 0);

  if (result == SFUD_SUCCESS)
  {
    result = sfud_read_status(flash, &register_status);
  }

  if (result == SFUD_SUCCESS)
  {
    if (enabled && (register_status & SFUD_STATUS_REGISTER_WEL) == 0)
    {
      return SFUD_ERR_WRITE;
    }
    else if (!enabled && (register_status & SFUD_STATUS_REGISTER_WEL) != 0)
    {
      return SFUD_ERR_WRITE;
    }
  }

  return result;
}

/*************************************************************************
 * enable or disable 4-Byte addressing for flash
 * @note The 4-Byte addressing just supported for the flash capacity which is large then 16MB (256Mb).
 * @param flash flash device
 * @param enabled true: enable   false: disable
 * @return result
*************************************************************************/
static sfud_status_t set_4_byte_address_mode(sfud_flash_t *flash, bool enabled)
{
  sfud_status_t result = SFUD_SUCCESS;
  uint8_t cmd;

  // set the flash write enable
  result = set_write_enabled(flash, true);
  if (result != SFUD_SUCCESS)
  {
    return result;
  }

  if (enabled)
  {
    cmd = SFUD_CMD_ENTER_4B_ADDRESS_MODE;
  }
  else
  {
    cmd = SFUD_CMD_EXIT_4B_ADDRESS_MODE;
  }

  result = flash->spi.rw(&flash->spi, &cmd, 1, NULL, 0);

  if (result == SFUD_SUCCESS)
  {
    flash->addr_in_4_byte = enabled ? true : false;
  }

  return result;
}

/*************************************************************************
 * read flash register status
 * @param flash flash device
 * @param status register status
 * @return result
*************************************************************************/
sfud_status_t sfud_read_status(const sfud_flash_t *flash, uint8_t *status)
{
  uint8_t cmd = SFUD_CMD_READ_STATUS_REGISTER;

  return flash->spi.rw(&flash->spi, &cmd, 1, status, 1);
}

static sfud_status_t wait_busy(const sfud_flash_t *flash)
{
  sfud_status_t result = SFUD_SUCCESS;
  uint8_t status;
  size_t retry_times = flash->retry.times;

  while (true)
  {
    result = sfud_read_status(flash, &status);
    if (result == SFUD_SUCCESS && ((status & SFUD_STATUS_REGISTER_BUSY)) == 0)
    {
      break;
    }
    // retry counts
    SFUD_RETRY_PROCESS(flash->retry.delay, retry_times, result);
  }

  return result;
}

//====================================================================================
static void make_adress_byte_array(const sfud_flash_t *flash, uint32_t addr, uint8_t *array)
{
  uint8_t len, i;

  len = flash->addr_in_4_byte ? 4 : 3;

  for (i = 0; i < len; i++)
  {
    array[i] = (addr >> ((len - (i + 1)) * 8)) & 0xFF;
  }
}

/*************************************************************************
 * write status register
 * @param flash flash device
 * @param is_volatile true: volatile mode, false: non-volatile mode
 * @param status register status
 * @return result
*************************************************************************/
sfud_status_t sfud_write_status(const sfud_flash_t *flash, bool is_volatile, uint8_t status)
{
  sfud_status_t result = SFUD_SUCCESS;
  const sfud_spi_t *spi = &flash->spi;
  uint8_t cmd_data[2];

  if (is_volatile)
  {
    cmd_data[0] = SFUD_VOLATILE_SR_WRITE_ENABLE;
    result = spi->rw(spi, cmd_data, 1, NULL, 0);
  }
  else
  {
    result = set_write_enabled(flash, true);
  }

  if (result == SFUD_SUCCESS)
  {
    cmd_data[0] = SFUD_CMD_WRITE_STATUS_REGISTER;
    cmd_data[1] = status;
    result = spi->rw(spi, cmd_data, 2, NULL, 0);
  }

  return result;
}

