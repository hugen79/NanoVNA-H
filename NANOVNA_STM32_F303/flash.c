static inline void flash_wait_for_last_operation(void)
{
  while (FLASH->SR == FLASH_SR_BSY) {
    //WWDG->CR = WWDG_CR_T;
  }
//  return FLASH->SR;
}

static void flash_erase_page0(uint32_t page_address)
{
  flash_wait_for_last_operation();
  FLASH->CR|= FLASH_CR_PER;
  FLASH->AR = page_address;
  FLASH->CR|= FLASH_CR_STRT;
  flash_wait_for_last_operation();
  FLASH->CR&=~FLASH_CR_PER;
}

static inline void flash_unlock(void)
{
  // unlock sequence
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

void flash_erase_pages(uint32_t page_address, uint32_t size)
{
  // Unlock for erase
  flash_unlock();
  // erase flash pages
  size+=page_address;
  for (; page_address < size; page_address+= FLASH_PAGESIZE)
    flash_erase_page0(page_address);
}

void flash_program_half_word_buffer(uint16_t* dst, uint16_t *data, uint16_t size)
{
  uint32_t i;
  // unlock, and erase flash pages for buffer (aligned to FLASH_PAGESIZE)
  flash_erase_pages((uint32_t)dst, size);
  // Save buffer
  __IO uint16_t* p = dst;
  for (i = 0; i < size/sizeof(uint16_t); i++){
    flash_wait_for_last_operation();
    FLASH->CR|= FLASH_CR_PG;
    p[i] = data[i];
    flash_wait_for_last_operation();
    FLASH->CR&=~FLASH_CR_PG;
  }
}
