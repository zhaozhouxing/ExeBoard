
#include "stm32_eval.h"

#include "bmp.h"

#define CONVERT_RGB(ucR,ucG,ucB) ((((ucR & 0XF8) >> 3) << 11)|(((ucG & 0XFC) >> 2) << 5)|(((ucB & 0XF8) >> 3) << 0))

void DrawBitmap(const uint8_t *bmpData,int x,int y,draw_pixel_callback cb)
{
  BmpSt *pBm = (BmpSt *)bmpData;

  uint8_t *pbmpCnt = (uint8_t *)bmpData + pBm->head.bfOffBits;

  int iRow,iCol;
  
  int iPitch =((pBm->info.biWidth * pBm->info.biBitCount + 31) / 32) * 4;

  for (iRow = 0; iRow < pBm->info.biHeight; iRow++)
  {   
      
      for (iCol = 0; iCol < pBm->info.biWidth; iCol++)
      {
         uint8_t data = 0 ;
         uint8_t ucByte,ucBits;
         uint16_t usColor;

         switch(pBm->info.biBitCount)
         {
         case 1:
            ucByte = iCol / 8;
            ucBits = iCol % 8;
            data = (pbmpCnt[ucByte] >> ((8 - 1) - ucBits)) &0x1;
            usColor = CONVERT_RGB((pBm->rgb[data].rgbRed), (pBm->rgb[data].rgbGreen),(pBm->rgb[data].rgbBlue));
            cb(x + iCol,(y + pBm->info.biHeight - 1 - iRow) ,usColor);
            
            break;
         case 2:
             ucByte = (iCol *2) / 8;
             ucBits = (iCol *2) % 8;
             data = (pbmpCnt[ucByte] >> ((8 - 2) - ucBits)) & 0x3;
             usColor = CONVERT_RGB((pBm->rgb[data].rgbRed), (pBm->rgb[data].rgbGreen),(pBm->rgb[data].rgbBlue));
             cb(x + iCol,(y + pBm->info.biHeight - 1 - iRow) ,usColor);
            
           break;
         case 4:
            ucByte = (iCol * 4) / 8;
            ucBits = (iCol * 4) % 8;
            data = (pbmpCnt[ucByte] >> (4 - ucBits)) & 0XF;
            usColor = CONVERT_RGB((pBm->rgb[data].rgbRed), (pBm->rgb[data].rgbGreen),(pBm->rgb[data].rgbBlue));
            cb(x + iCol,(y + pBm->info.biHeight - 1 - iRow) ,usColor);

            break;
         case 8:
            ucByte = iCol;
            data = pbmpCnt[ucByte] ;
            usColor = CONVERT_RGB((pBm->rgb[data].rgbRed), (pBm->rgb[data].rgbGreen),(pBm->rgb[data].rgbBlue));
            cb(x + iCol,(y + pBm->info.biHeight - 1 - iRow) ,usColor);
            break;
         default:
            return ;
         } 
      }
      pbmpCnt += iPitch;
  
  }
  
   
}
