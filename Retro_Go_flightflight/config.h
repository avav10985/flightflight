// ============================================================
// Retro-Go target — flightflight 手把 V2-A(ESP32-S3 N16R8)
//
// 腳位全部沿用手把現有接線(同 Ground_TX_ESP32 / NES_Test):
//   顯示 ILI9341 on SPI2、SD on SPI3、MAX98357A I2S DAC
//   輸入:雙搖桿 ADC(方向)+ 按鍵叢電阻階梯 ADC + 雙肩鈕
//
// 操作對應:
//   右搖桿 Y(GPIO4)= 上下      左搖桿 X(GPIO2)= 左右
//   右肩鈕(GPIO9)= A           左肩鈕(GPIO8)= MENU(retro-go 選單/存檔)
//   按鍵叢(GPIO7):+ = B、OK = START、返回 = SELECT
// ============================================================

// Target definition
#define RG_TARGET_NAME             "FLIGHTFLIGHT"

// Storage(SD on SPI3,同 NES_Test)
#define RG_STORAGE_ROOT             "/sd"
#define RG_STORAGE_SDSPI_HOST       SPI3_HOST
#define RG_STORAGE_SDSPI_SPEED      SDMMC_FREQ_DEFAULT

// Audio(MAX98357A 外部 I2S DAC)
#define RG_AUDIO_USE_INT_DAC        0
#define RG_AUDIO_USE_EXT_DAC        1

// Video(ILI9341,SPI2,橫向 320x240)
#define RG_SCREEN_DRIVER            0   // 0 = ILI9341/ST7789
#define RG_SCREEN_HOST              SPI2_HOST
#define RG_SCREEN_SPEED             SPI_MASTER_FREQ_40M
#define RG_SCREEN_BACKLIGHT         0   // 背光直連 3V3 軌常亮,無 GPIO 控制
#define RG_SCREEN_WIDTH             320
#define RG_SCREEN_HEIGHT            240
#define RG_SCREEN_ROTATE            0
#define RG_SCREEN_VISIBLE_AREA      {0, 0, 0, 0}
#define RG_SCREEN_SAFE_AREA         {0, 0, 0, 0}
#define RG_SCREEN_INIT()                                                                                         \
    ILI9341_CMD(0xCF, 0x00, 0xc3, 0x30);                                                                         \
    ILI9341_CMD(0xED, 0x64, 0x03, 0x12, 0x81);                                                                   \
    ILI9341_CMD(0xE8, 0x85, 0x00, 0x78);                                                                         \
    ILI9341_CMD(0xCB, 0x39, 0x2c, 0x00, 0x34, 0x02);                                                             \
    ILI9341_CMD(0xF7, 0x20);                                                                                     \
    ILI9341_CMD(0xEA, 0x00, 0x00);                                                                               \
    ILI9341_CMD(0xC0, 0x1B);                                                                                     \
    ILI9341_CMD(0xC1, 0x12);                                                                                     \
    ILI9341_CMD(0xC5, 0x32, 0x3C);                                                                               \
    ILI9341_CMD(0xC7, 0x91);                                                                                     \
    ILI9341_CMD(0x36, 0x68);                 /* Memory Access Control(橫向;畫面方向不對改這個 byte)*/          \
    ILI9341_CMD(0xB1, 0x00, 0x10);                                                                               \
    ILI9341_CMD(0xB6, 0x0A, 0xA2);                                                                               \
    ILI9341_CMD(0xF6, 0x01, 0x30);                                                                               \
    ILI9341_CMD(0xF2, 0x00);                                                                                     \
    ILI9341_CMD(0x26, 0x01);                                                                                     \
    ILI9341_CMD(0xE0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00); \
    ILI9341_CMD(0xE1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F);


// Input — 方向走搖桿 ADC,動作鍵走按鍵叢 ADC + 肩鈕 GPIO
// 搖桿軸向跟手把一致(反相):右搖桿推上=ADC 高、左搖桿推右=ADC 低
// ESP32-S3 ADC1:GPIO2=CH1、GPIO4=CH3、GPIO7=CH6
#define RG_GAMEPAD_ADC_MAP {\
    {RG_KEY_UP,     ADC_UNIT_1, ADC_CHANNEL_3, ADC_ATTEN_DB_11, 3072, 4095},\
    {RG_KEY_DOWN,   ADC_UNIT_1, ADC_CHANNEL_3, ADC_ATTEN_DB_11,    0, 1024},\
    {RG_KEY_RIGHT,  ADC_UNIT_1, ADC_CHANNEL_1, ADC_ATTEN_DB_11,    0, 1024},\
    {RG_KEY_LEFT,   ADC_UNIT_1, ADC_CHANNEL_1, ADC_ATTEN_DB_11, 3072, 4095},\
    {RG_KEY_B,      ADC_UNIT_1, ADC_CHANNEL_6, ADC_ATTEN_DB_11, 3300, 4095},\
    {RG_KEY_START,  ADC_UNIT_1, ADC_CHANNEL_6, ADC_ATTEN_DB_11, 1550, 2300},\
    {RG_KEY_SELECT, ADC_UNIT_1, ADC_CHANNEL_6, ADC_ATTEN_DB_11,  600, 1549},\
}
#define RG_GAMEPAD_GPIO_MAP {\
    {RG_KEY_A,    .num = GPIO_NUM_9, .pullup = 1, .level = 0},\
    {RG_KEY_MENU, .num = GPIO_NUM_8, .pullup = 1, .level = 0},\
}

// Battery(手把無電池分壓,停用)
#define RG_BATTERY_DRIVER           0

// SPI Display(ILI9341,SPI2)
#define RG_GPIO_LCD_MISO            GPIO_NUM_40
#define RG_GPIO_LCD_MOSI            GPIO_NUM_39
#define RG_GPIO_LCD_CLK             GPIO_NUM_38
#define RG_GPIO_LCD_CS              GPIO_NUM_48
#define RG_GPIO_LCD_DC              GPIO_NUM_21
#define RG_GPIO_LCD_BCKL            GPIO_NUM_NC
#define RG_GPIO_LCD_RST             GPIO_NUM_16

// SD card(SPI3,獨立 bus)
#define RG_GPIO_SDSPI_MISO          GPIO_NUM_47
#define RG_GPIO_SDSPI_MOSI          GPIO_NUM_17
#define RG_GPIO_SDSPI_CLK           GPIO_NUM_15
#define RG_GPIO_SDSPI_CS            GPIO_NUM_0

// External I2S DAC(MAX98357A)
#define RG_GPIO_SND_I2S_BCK         11
#define RG_GPIO_SND_I2S_WS          12
#define RG_GPIO_SND_I2S_DATA        13
#define RG_GPIO_SND_AMP_ENABLE      18
