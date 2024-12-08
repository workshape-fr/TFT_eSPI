        ////////////////////////////////////////////////////
        //       TFT_eSPI generic driver functions        //
        ////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////////////

// Select the SPI port to use
#ifdef TFT_SPI_PORT
  SPIClass& spi = TFT_SPI_PORT;
#else
  SPIClass& spi = SPI;
  nrfx_spim_t spim3 = NRFX_SPIM_INSTANCE(3);
#endif

////////////////////////////////////////////////////////////////////////////////////////
#if defined (TFT_SDA_READ) && !defined (TFT_PARALLEL_8_BIT)
////////////////////////////////////////////////////////////////////////////////////////

/***************************************************************************************
** Function name:           tft_Read_8
** Description:             Bit bashed SPI to read bidirectional SDA line
***************************************************************************************/
uint8_t TFT_eSPI::tft_Read_8(void)
{
  uint8_t  ret = 0;

  for (uint8_t i = 0; i < 8; i++) {  // read results
    ret <<= 1;
    SCLK_L;
    if (digitalRead(TFT_MOSI)) ret |= 1;
    SCLK_H;
  }

  return ret;
}

/***************************************************************************************
** Function name:           beginSDA
** Description:             Detach SPI from pin to permit software SPI
***************************************************************************************/
void TFT_eSPI::begin_SDA_Read(void)
{
  // Release configured SPI port for SDA read
  spi.end();
}

/***************************************************************************************
** Function name:           endSDA
** Description:             Attach SPI pins after software SPI
***************************************************************************************/
void TFT_eSPI::end_SDA_Read(void)
{
  // Configure SPI port ready for next TFT access
  spi.begin();
}

////////////////////////////////////////////////////////////////////////////////////////
#endif // #if defined (TFT_SDA_READ)
////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////
#if defined (TFT_PARALLEL_8_BIT) // Code for generic (i.e. any) processor
////////////////////////////////////////////////////////////////////////////////////////

/***************************************************************************************
** Function name:           pushBlock - for generic processor and parallel display
** Description:             Write a block of pixels of the same colour
***************************************************************************************/
void TFT_eSPI::pushBlock(uint16_t color, uint32_t len){

  while (len>1) {tft_Write_32D(color); len-=2;}
  if (len) {tft_Write_16(color);}
}

/***************************************************************************************
** Function name:           pushPixels - for gereric processor and parallel display
** Description:             Write a sequence of pixels
***************************************************************************************/
void TFT_eSPI::pushPixels(const void* data_in, uint32_t len){

  uint16_t *data = (uint16_t*)data_in;
  if(_swapBytes) {
    while (len>1) {tft_Write_16(*data); data++; tft_Write_16(*data); data++; len -=2;}
    if (len) {tft_Write_16(*data);}
    return;
  }

  while (len>1) {tft_Write_16S(*data); data++; tft_Write_16S(*data); data++; len -=2;}
  if (len) {tft_Write_16S(*data);}
}

/***************************************************************************************
** Function name:           GPIO direction control  - supports class functions
** Description:             Set parallel bus to INPUT or OUTPUT
***************************************************************************************/
void TFT_eSPI::busDir(uint32_t mask, uint8_t mode)
{
  // mask is unused for generic processor
  // Arduino native functions suited well to a generic driver
  pinMode(TFT_D0, mode);
  pinMode(TFT_D1, mode);
  pinMode(TFT_D2, mode);
  pinMode(TFT_D3, mode);
  pinMode(TFT_D4, mode);
  pinMode(TFT_D5, mode);
  pinMode(TFT_D6, mode);
  pinMode(TFT_D7, mode);
  return;
}

/***************************************************************************************
** Function name:           GPIO direction control  - supports class functions
** Description:             Faster GPIO pin input/output switch
***************************************************************************************/
void TFT_eSPI::gpioMode(uint8_t gpio, uint8_t mode)
{
  // No fast port based generic approach available
}

/***************************************************************************************
** Function name:           read byte  - supports class functions
** Description:             Read a byte - parallel bus only
***************************************************************************************/
uint8_t TFT_eSPI::readByte(void)
{
  uint8_t b = 0;

  busDir(0, INPUT);
  digitalWrite(TFT_RD, LOW);

  b |= digitalRead(TFT_D0) << 0;
  b |= digitalRead(TFT_D1) << 1;
  b |= digitalRead(TFT_D2) << 2;
  b |= digitalRead(TFT_D3) << 3;
  b |= digitalRead(TFT_D4) << 4;
  b |= digitalRead(TFT_D5) << 5;
  b |= digitalRead(TFT_D6) << 6;
  b |= digitalRead(TFT_D7) << 7;

  digitalWrite(TFT_RD, HIGH);
  busDir(0, OUTPUT); 

  return b;
}

////////////////////////////////////////////////////////////////////////////////////////
#elif defined (RPI_WRITE_STROBE)  // For RPi TFT with write strobe                      
////////////////////////////////////////////////////////////////////////////////////////

/***************************************************************************************
** Function name:           pushBlock - for ESP32 or STM32 RPi TFT
** Description:             Write a block of pixels of the same colour
***************************************************************************************/
void TFT_eSPI::pushBlock(uint16_t color, uint32_t len){

  if(len) { tft_Write_16(color); len--; }
  while(len--) {WR_L; WR_H;}
}

/***************************************************************************************
** Function name:           pushPixels - for ESP32 or STM32 RPi TFT
** Description:             Write a sequence of pixels
***************************************************************************************/
void TFT_eSPI::pushPixels(const void* data_in, uint32_t len)
{
  uint16_t *data = (uint16_t*)data_in;

  if (_swapBytes) while ( len-- ) {tft_Write_16S(*data); data++;}
  else while ( len-- ) {tft_Write_16(*data); data++;}
}

////////////////////////////////////////////////////////////////////////////////////////
#elif defined (SPI_18BIT_DRIVER) // SPI 18-bit colour                         
////////////////////////////////////////////////////////////////////////////////////////

/***************************************************************************************
** Function name:           pushBlock - for STM32 and 3 byte RGB display
** Description:             Write a block of pixels of the same colour
***************************************************************************************/
void TFT_eSPI::pushBlock(uint16_t color, uint32_t len)
{
  // Split out the colours
  uint8_t r = (color & 0xF800)>>8;
  uint8_t g = (color & 0x07E0)>>3;
  uint8_t b = (color & 0x001F)<<3;

  while ( len-- ) {tft_Write_8(r); tft_Write_8(g); tft_Write_8(b);}
}

/***************************************************************************************
** Function name:           pushPixels - for STM32 and 3 byte RGB display
** Description:             Write a sequence of pixels
***************************************************************************************/
void TFT_eSPI::pushPixels(const void* data_in, uint32_t len){

  uint16_t *data = (uint16_t*)data_in;
  if (_swapBytes) {
    while ( len-- ) {
      uint16_t color = *data >> 8 | *data << 8;
      tft_Write_8((color & 0xF800)>>8);
      tft_Write_8((color & 0x07E0)>>3);
      tft_Write_8((color & 0x001F)<<3);
      data++;
    }
  }
  else {
    while ( len-- ) {
      tft_Write_8((*data & 0xF800)>>8);
      tft_Write_8((*data & 0x07E0)>>3);
      tft_Write_8((*data & 0x001F)<<3);
      data++;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
#else //                   Standard SPI 16-bit colour TFT                               
////////////////////////////////////////////////////////////////////////////////////////

/***************************************************************************************
** Function name:           pushBlock - for STM32
** Description:             Write a block of pixels of the same colour
***************************************************************************************/
void TFT_eSPI::pushBlock(uint16_t color, uint32_t len){

  while ( len-- ) {tft_Write_16(color);}
}

/***************************************************************************************
** Function name:           pushPixels - for STM32
** Description:             Write a sequence of pixels
***************************************************************************************/
void TFT_eSPI::pushPixels(const void* data_in, uint32_t len){

  uint16_t *data = (uint16_t*)data_in;

  if (_swapBytes) while ( len-- ) {tft_Write_16(*data); data++;}
  else while ( len-- ) {tft_Write_16S(*data); data++;}
}

////////////////////////////////////////////////////////////////////////////////////////
#endif // End of display interface specific functions
////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////
//                                DMA FUNCTIONS                                         
////////////////////////////////////////////////////////////////////////////////////////

//                Placeholder for DMA functions

/*
Minimal function set to support DMA:

bool TFT_eSPI::initDMA(void)
void TFT_eSPI::deInitDMA(void)
bool TFT_eSPI::dmaBusy(void)
void TFT_eSPI::pushPixelsDMA(uint16_t* image, uint32_t len)
void TFT_eSPI::pushImageDMA(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* image)

*/

bool TFT_eSPI::initDMA(bool ctrl_cs) {
  // DMA is already initialized on SPI3
  DMA_Enabled = true;
  return true;
}

void TFT_eSPI::pushPixelsDMA(uint16_t* image, uint32_t len) {
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(image, len);

  if ((len == 0) || (!DMA_Enabled)) return;

  dmaWait();

  (void)nrfx_spim_xfer(&spim3, &xfer_desc, 0);
}

void TFT_eSPI::pushImageDMA(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* image, uint16_t* buffer) {
  if ((x >= _vpW) || (y >= _vpH) || (!DMA_Enabled)) return;

  _swapBytes = true;

  int32_t dx = 0;
  int32_t dy = 0;
  int32_t dw = w;
  int32_t dh = h;

  if (x < _vpX) { dx = _vpX - x; dw -= dx; x = _vpX; }
  if (y < _vpY) { dy = _vpY - y; dh -= dy; y = _vpY; }

  if ((x + dw) > _vpW ) dw = _vpW - x;
  if ((y + dh) > _vpH ) dh = _vpH - y;

  if (dw < 1 || dh < 1) return;

  uint32_t len = dw*dh;

  if (buffer == nullptr) {
    buffer = image;
    dmaWait();
  }

  // If image is clipped, copy pixels into a contiguous block
  if ( (dw != w) || (dh != h) ) {
    for (int32_t yb = 0; yb < dh; yb++) {
      memmove((uint8_t*) (buffer + yb * dw), (uint8_t*) (image + dx + w * (yb + dy)), dw << 1);
    }
  }
  // else, if a buffer pointer has been provided copy whole image to the buffer
  else if (buffer != image || _swapBytes) {
    memcpy(buffer, image, len*2);
  }

  dmaWait(); // In case we did not wait earlier

  setAddrWindow(x, y, dw, dh);
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(buffer, (size_t)(2 * dw * dh));
  (void)nrfx_spim_xfer(&spim3, &xfer_desc, 0);
}

void TFT_eSPI::dmaWait(void) {
  // Wait for the SPIM_END event to indicate transfer completion
  int time = millis();
  // SPI clock: 8–32 MHz → Timeout: 1ms
  while (!nrf_spim_event_check(spim3.p_reg, NRF_SPIM_EVENT_END)) {
    // Optionally, add a timeout mechanism to avoid infinite blocking
    if (millis() >= time + 1)
      break;
  }

  // Clear the event to prepare for the next transfer
  nrf_spim_event_clear(spim3.p_reg, NRF_SPIM_EVENT_END);
}

bool TFT_eSPI::dmaBusy(void) {
  return !nrf_spim_event_check(spim3.p_reg, NRF_SPIM_EVENT_END);
}
