//==============================================================================
//  Copyright 2011-2013 Meta Watch Ltd. - http://www.MetaWatch.org/
//
//  Licensed under the Meta Watch License, Version 1.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.MetaWatch.org/licenses/license-1.0.html
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//==============================================================================

/******************************************************************************/
/*! \file SerialRam.c
*
*/
/******************************************************************************/

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "hal_board_type.h"
#include "hal_clock_control.h"
#include "hal_miscellaneous.h"
#include "hal_battery.h"
#include "hal_lcd.h"
#include "hal_rtos_timer.h"
#include "Messages.h"
#include "MessageQueues.h"
#include "DebugUart.h"
#include "SerialRam.h"
#include "LcdDriver.h"
#include "LcdDisplay.h"
#include "Fonts.h"
#include "LcdBuffer.h"
#include "Utilities.h"
#include "Adc.h"
#include "BitmapData.h"
#include "Wrapper.h"
#include "Property.h"

/******************************************************************************/

#define SPI_READ  ( 0x03 )
#define SPI_WRITE ( 0x02 )
/* write and read status register */
#define SPI_RDSR  ( 0x05 )
#define SPI_WRSR  ( 0x01 )
#define SPI_OVERHEAD ( 3 )
#define SPI_INIT_DELAY_IN_MS (10 * portTICK_RATE_MS)

/* the 256Kbit part does not have a 1 in bit position 1 */
#define DEFAULT_SR_VALUE        (0x02)
#define FINAL_SR_VALUE          (0x43)
#define DEFAULT_SR_VALUE_256    (0x00)
#define FINAL_SR_VALUE_256      (0x41)
#define SEQUENTIAL_MODE_COMMAND (0x41)

#define MAX_WIDGET_NUM          (16)
#define QUAD_NO_MASK            (0x03)
#define IDLE_PAGE_MASK          (0x30)
#define IDLE_PAGE_SHFT          (4)
#define IDLE_PAGE_NUM           (4)

#define MODE_BUF_START_ADDR     (0x0E00) // 1152x3 = 3456
#define MODE_BUFFER_SIZE        (0x1200) // 1152x4
#define WGT_BUF_START_ADDR      (0x80) //(1024x8-1152x7=128 (0x1B00) // 96x12x(4 + 2) = 6912

#define WRITE_DATA_LEN          (BYTES_PER_QUAD_LINE + BYTES_PER_QUAD_LINE)
#define SRAM_DATA_LEN           (SRAM_HEADER_LEN + WRITE_DATA_LEN)
#define SRAM_BUFFER_SIZE        (SRAM_HEADER_LEN + BYTES_PER_LINE)
#define BUFFER_TAG_BITS         (16)
#define WGTLST_PART_INDX_MASK   (0x3)
#define WGTLST_PARTS_MASK       (0xC)
#define WGTLST_PART_INDX_SHFT   (0)
#define WGTLST_PARTS_SHFT       (2)
#define INVALID_ID              (0xFF)
#define CLOCK_WIDGET_ID_RANGE   (15) // home widget: 0 - 15
#define INVERT_BIT              (BIT6)
#define CLOCK_WIDGET_BIT        (BIT7)
#define CLOCK_WIDGET_BIT_SHFT   (7)
#define WGT_CHG_REMOVE          (0)
#define WGT_CHG_ADD             (1)
#define WGT_CHG_CLK_ADD         (2)
#define WGT_CHG_CLK_FACE        (3)
#define WGT_CHG_SETTING         (4)

#define BOARDER_PATTERN_ROW     (0x66)
#define BOARDER_PATTERN_COL     (4)

#define WRITE_BUFFER_TWO_LINES     (0x00)
#define MSG_OPT_WRTBUF_BPL_MASK    (0x38)

/* errata - DMA variables cannot be function scope */
static const unsigned char DummyData = 0x00;
static unsigned char ReadData = 0x00;
static unsigned char DmaBusy  = 0;

static signed char CurrentPage = 0;

static xSemaphoreHandle SramMutex;

typedef struct
{
  unsigned char StartRow;
  unsigned char RowNum;
} Rect_t;

typedef struct
{
  unsigned char Id;
  unsigned char Layout;
} WidgetList_t;

#define WIDGET_HEADER_LEN  (sizeof(WidgetList_t))

typedef struct
{
  unsigned char Id;
  unsigned char Layout;
  unsigned char Outdated;
  unsigned char Buffers[QUAD_NUM];
} Widget_t;

static Widget_t Widget[MAX_WIDGET_NUM + MAX_WIDGET_NUM];
static Widget_t *pCurrWidgetList = &Widget[MAX_WIDGET_NUM]; // point to left/right half of double sized Widget[]

const Layout_t Layout[] = {{1, 0}, {2, 1}, {2, 2}, {4, 1}};

static unsigned int BufTag = 0;
///* low 4 bits: Clock widget ID; high 4 bits: to be updated if set */
//static unsigned char ClkWgtUpd[CLOCK_WIDGET_ID_RANGE + 1];

typedef struct
{
  unsigned char Id;
  unsigned char Row;
} WidgetHeader_t;

typedef struct
{
  unsigned char Page;
  unsigned int Addr[QUAD_NUM];
  unsigned char Layout[QUAD_NUM];
} QuadAddr_t;

static const unsigned char ModePriority[] = {NOTIF_MODE, APP_MODE, IDLE_MODE, MUSIC_MODE};

#define IS_CLOCK_WIDGET(_x) ((_x & CLOCK_WIDGET_BIT) >> CLOCK_WIDGET_BIT_SHFT)
#define ON_CURRENT_PAGE(_x) ((_x & IDLE_PAGE_MASK) >> IDLE_PAGE_SHFT == CurrentPage)
#define LAYOUT_TYPE(_x) ((_x & LAYOUT_MASK) >> LAYOUT_SHFT)
#define WGTLST_INDEX(_x) ((_x & WGTLST_PART_INDX_MASK) >> WGTLST_PART_INDX_SHFT)
#define WGTLST_TOTAL(_x) ((_x & WGTLST_PARTS_MASK) >> WGTLST_PARTS_SHFT)

/******************************************************************************/
static void SetupCycle(unsigned int Address,unsigned char CycleType);
static void WriteBlockToSram(const unsigned char* pData,unsigned int Size);
static void ReadBlock(unsigned char* pWriteData,unsigned char* pReadData);
static void LoadBuffer(unsigned char i, const unsigned char *pTemp);
static void ClearSram(unsigned int Address, unsigned char Data, unsigned int Size);

static void AssignWidgetBuffer(Widget_t *pWidget);
static void FreeWidgetBuffer(Widget_t *pWidget);
static unsigned int GetAddr(WidgetHeader_t *pData);
static void GetQuadAddr(QuadAddr_t *pAddr);
static void WriteClockWidget(unsigned char *pBuffer);
static signed char ComparePriority(unsigned char Mode);
static unsigned char GetWidgetChange(unsigned char CurId, unsigned char CurOpt, unsigned char MsgId, unsigned char MsgOpt);
static void TestFaceId(WidgetList_t *pWidget);

#if __IAR_SYSTEMS_ICC__
static void WriteData20BlockToSram(unsigned char __data20* pData,unsigned int Size);
#endif

void SetWidgetList(tMessage *pMsg)
{
  static Widget_t *pCurrWidget = NULL; // point to Widget in current Widget[]
  static Widget_t *pNextWidget = NULL; // point to Widget in new Widget[]
  static unsigned char ChangedClockWidget = INVALID_ID;

  xSemaphoreTake(SramMutex, portMAX_DELAY);

  WidgetList_t *pMsgWgtLst = (WidgetList_t *)pMsg->pBuffer;
  unsigned char WidgetNum = pMsg->Length / WIDGET_HEADER_LEN;

  unsigned char i = 0;
  PrintF(">SetWLst I:%d %s %d %s %d", WGTLST_INDEX(pMsg->Options), "T:", WGTLST_TOTAL(pMsg->Options), "Num:", WidgetNum);
  for(; i<WidgetNum; ++i) {PrintH(pMsgWgtLst[i].Id); PrintH(pMsgWgtLst[i].Layout);} PrintR();

  if (pNextWidget == NULL) // first time call, only add widgets
  {
    pCurrWidget = pCurrWidgetList;
    pNextWidget = &Widget[0];
  }
  else
  {
    if (WGTLST_INDEX(pMsg->Options) == 0 &&
      (pCurrWidget != pCurrWidgetList || (pNextWidget != &Widget[0] && pNextWidget != &Widget[MAX_WIDGET_NUM])))
    { // last SetWLst failed in the middle.Clean up whole list
      PrintS("# Last SetWgtLst broken!");

      pCurrWidget = pCurrWidgetList;
      pNextWidget = &Widget[0] + (&Widget[MAX_WIDGET_NUM] - pCurrWidgetList);
    }
  }

  while (WidgetNum) // number of list items
  {
      /* old clock widgets */
    if (!IS_CLOCK_WIDGET(pMsgWgtLst->Layout) && pMsgWgtLst->Id <= CLOCK_WIDGET_ID_RANGE) TestFaceId(pMsgWgtLst);
    unsigned char Change = GetWidgetChange(pCurrWidget->Id, pCurrWidget->Layout, pMsgWgtLst->Id, pMsgWgtLst->Layout);
    
    switch (Change)
    {
    case WGT_CHG_CLK_FACE:
      PrintS("Chg ClkFce");
      if (ON_CURRENT_PAGE(pMsgWgtLst->Layout)) ChangedClockWidget = pMsgWgtLst->Id;
      
    case WGT_CHG_SETTING:
     //cpy layout to curr; cpy curr to next; msg, curr, next ++
      PrintF("=%02X", pCurrWidget->Id);
      pCurrWidget->Id = pMsgWgtLst->Id;
      pCurrWidget->Layout = pMsgWgtLst->Layout;
      *pNextWidget++ = *pCurrWidget++;
      pMsgWgtLst ++;
      WidgetNum --;
      break;

    case WGT_CHG_CLK_ADD:
      PrintS("+Clk");
      if (ON_CURRENT_PAGE(pMsgWgtLst->Layout)) ChangedClockWidget = pMsgWgtLst->Id;

    case WGT_CHG_ADD: //pCurrWidget->Id > pMsgWgtLst->Id)
     // add new widget: cpy msg to next; msg and next ++; curr stays
      PrintF("+%02X", pMsgWgtLst->Id);

      pNextWidget->Id = pMsgWgtLst->Id;
      pNextWidget->Layout = pMsgWgtLst->Layout;
      AssignWidgetBuffer(pNextWidget);

      pNextWidget ++;
      pMsgWgtLst ++;
      WidgetNum --;
      break;
      
    case WGT_CHG_REMOVE:
    // remove widget: curr ++
      PrintF("-%02X", pCurrWidget->Id);
      FreeWidgetBuffer(pCurrWidget);
      pCurrWidget ++;
      break;
      
    default: break;
    }
  }
  PrintR();

  // if part index + 1 == parts, SetWidgetList complete
  if (WGTLST_TOTAL(pMsg->Options) == WGTLST_INDEX(pMsg->Options) + 1)
  {
//    PrintS("C:");
//    for (i=0; pCurrWidgetList[i].Id != INVALID_ID && i < MAX_WIDGET_NUM; ++i) PrintH(pCurrWidgetList[i].Id);
//    PrintR();

    while (pCurrWidget->Id != INVALID_ID && pCurrWidget < &pCurrWidgetList[MAX_WIDGET_NUM])
    {
      FreeWidgetBuffer(pCurrWidget);
      pCurrWidget->Id = INVALID_ID;
      pCurrWidget ++;
    }

    for (i = 0; i < MAX_WIDGET_NUM; ++i)
    {
      if (pCurrWidgetList[i].Id != INVALID_ID)
      { // clear the widget id in the curr list
        pCurrWidgetList[i].Id = INVALID_ID;
      }
    }

    pNextWidget = pCurrWidgetList;
    pCurrWidgetList = &Widget[0] + (&Widget[MAX_WIDGET_NUM] - pCurrWidgetList);
    pCurrWidget = pCurrWidgetList;

//    PrintS("N:");
//    for (i=0; pCurrWidgetList[i].Id != INVALID_ID; ++i) PrintH(pCurrWidgetList[i].Id);
//    PrintR();
    PrintF("Tg:%04X", BufTag);

    if (ChangedClockWidget != INVALID_ID)
    {
      CreateAndSendMessage(DrawClockWidgetMsg, ChangedClockWidget);
      ChangedClockWidget = INVALID_ID;
    }
  }
  xSemaphoreGive(SramMutex);
}

static unsigned char GetWidgetChange(unsigned char CurId, unsigned char CurOpt,
                                     unsigned char MsgId, unsigned char MsgOpt)
{
  unsigned char CurFaceId = FACE_ID(CurId);
  unsigned char MsgFaceId = FACE_ID(MsgId);
  unsigned char Change;

  if (IS_CLOCK_WIDGET(CurOpt)) CurId = CurId & 0x0F;
  if (IS_CLOCK_WIDGET(MsgOpt)) MsgId = MsgId & 0x0F;

//  if (IS_CLOCK_WIDGET(MsgOpt))
//  {
//    unsigned char CurrWgtId = CurrId & 0x0F;
//    unsigned char MsgWgtId = MsgId & 0x0F;
//
//    if (CurrWgtId < MsgWgtId) return WGT_CHG_REMOVE;
//    if (CurrWgtId > MsgWgtId) return WGT_CHG_CLK_ADD;
//    if (CurrWgtId == MsgWgtId && FACE_ID(CurrId) != FACE_ID(MsgId))
//      return WGT_CHG_CLK_FACE;
//  }

  if (CurId == MsgId)
  {
    Change = IS_CLOCK_WIDGET(CurOpt) && CurFaceId != MsgFaceId ?
      WGT_CHG_CLK_FACE : WGT_CHG_SETTING;
  }
  else if (CurId < MsgId) Change = WGT_CHG_REMOVE;
  else Change = IS_CLOCK_WIDGET(MsgOpt) ? WGT_CHG_CLK_ADD : WGT_CHG_ADD;
  return Change;
}

static void TestFaceId(WidgetList_t *pWidget)
{
  PrintF("Clk BF: Id:0x%02X Layout:0x%02X", pWidget->Id, pWidget->Layout);
  
  // copy layout type to faceId,
  pWidget->Id |= LAYOUT_TYPE(pWidget->Layout) << 4;
  // if type 3, invert-> faceId 4
//  if (LAYOUT_TYPE(pWidget->Layout) == LAYOUT_FULL_SCREEN && (pWidget->Layout & INVERT_BIT))
//  {
//    pWidget->Layout &= ~INVERT_BIT;
//    pWidget->Id = (((pWidget->Id >> 4) + 1) << 4) | (pWidget->Id & 0x0F);
//  }

  // set clock widget bit
  pWidget->Layout |= CLOCK_WIDGET_BIT;
  PrintF("Clk AF: Id:0x%02X Layout:0x%02X", pWidget->Id, pWidget->Layout);
}

static void AssignWidgetBuffer(Widget_t *pWidget)
{
  unsigned QuadNum = Layout[LAYOUT_TYPE(pWidget->Layout)].QuadNum;
  unsigned char i;

  for (i = 0; i < QuadNum; ++i)
  {
    unsigned char Tag;
    for (Tag = 0; Tag < BUFFER_TAG_BITS; ++Tag) // go through 16 bufs
    {
      if ((BufTag & (1 << Tag)) == 0) // found empty buffer
      {
        pWidget->Buffers[i] = Tag;
        BufTag |= 1 << Tag;

      // add "..." template
        LoadBuffer(Tag, pWidgetTemplate[TMPL_WGT_LOADING]);
        break;
      }
    }
  }
}

static void FreeWidgetBuffer(Widget_t *pWidget)
{
  unsigned QuadNum = Layout[LAYOUT_TYPE(pWidget->Layout)].QuadNum;
  unsigned char i;

  for (i = 0; i < QuadNum; ++i)
  {
    BufTag &= ~(1 << pWidget->Buffers[i]);
    // add "+" template to empty buffer
    LoadBuffer(pWidget->Buffers[i], pWidgetTemplate[TMPL_WGT_EMPTY]);
  }

  PrintF("-Tg:%04X", BufTag);
}

//#define MSG_OPT_NEWUI             (0x80)
//#define MSG_OPT_HOME_WGT          (0x40) // new ui only
//#define MODE_MASK                 (0x03)
//#define MSG_OPT_WRTBUF_1_LINE     (0x10)
//#define WRITE_BUFFER_TWO_LINES    (0x00)
//#define IDLE_MODE_PAGE_MASK       (0x07)
//#define MSG_OPT_WRTBUF_MULTILINE  (0x40) // for music mode
//#define MSG_OPT_WRTBUF_BPL_MASK   (0x38) // for music mode

void WriteBufferHandler(tMessage* pMsg)
{
  if (pMsg->Options & MSG_OPT_NEWUI)
  {
    if (pMsg->Options & MSG_OPT_HOME_WGT)
    {
      WriteClockWidget(pMsg->pBuffer);
      pMsg->pBuffer = NULL; // the pBuffer here is not allocated in message queue
    }
    else
    {
      unsigned int Addr = GetAddr((WidgetHeader_t *)(pMsg->pBuffer));
      unsigned char *pBuffer = pMsg->pBuffer + WIDGET_HEADER_LEN - SRAM_HEADER_LEN;
      pBuffer[0] = SPI_WRITE;
      pBuffer[1] = Addr >> 8;
      pBuffer[2] = Addr;

      WriteBlockToSram(pBuffer, SRAM_DATA_LEN);
    }
  }
  else
  {
    unsigned int Addr = (pMsg->Options & IDLE_MODE_PAGE_MASK) * BYTES_PER_SCREEN +
                        *pMsg->pBuffer * BYTES_PER_LINE + MODE_BUF_START_ADDR;
    
    unsigned char *pBuffer = pMsg->pBuffer + 1 - SRAM_HEADER_LEN;
    unsigned char BytesPerLine = pMsg->Options & MSG_OPT_WRTBUF_MULTILINE ?
                                (pMsg->Options & MSG_OPT_WRTBUF_BPL_MASK) >> 3 :
                                 BYTES_PER_LINE;
    
    unsigned char i = (pMsg->Length - 1) / BytesPerLine;
    if (BytesPerLine != BYTES_PER_LINE) PrintF("- Wrtbuf BPL:%d %s %d", BytesPerLine, "Ln:", i);
    
    while (i--)
    {
      pBuffer[0] = SPI_WRITE;
      pBuffer[1] = Addr >> 8;
      pBuffer[2] = Addr;
      WriteBlockToSram(pBuffer, SRAM_HEADER_LEN + BytesPerLine);

      Addr += BYTES_PER_LINE;
      pBuffer += BytesPerLine + (BytesPerLine == BYTES_PER_LINE);
    }
  }
}

/* mark all clock widgets outdated and request for updating
   the first widget on current page */
void UpdateClockWidgets(void)
{
  unsigned char i;
  unsigned char NotUpdate = pdTRUE;

  xSemaphoreTake(SramMutex, portMAX_DELAY);

  for (i = 0; pCurrWidgetList[i].Id != INVALID_ID && i < MAX_WIDGET_NUM; ++i)
  {
    if (!IS_CLOCK_WIDGET(pCurrWidgetList[i].Layout)) continue;

    pCurrWidgetList[i].Outdated = pdTRUE;
    if (NotUpdate && ON_CURRENT_PAGE(pCurrWidgetList[i].Layout))
    {
      CreateAndSendMessage(DrawClockWidgetMsg, pCurrWidgetList[i].Id);
      NotUpdate = pdFALSE;
    }
  }

  xSemaphoreGive(SramMutex);
}

/* get widget ID of the next outdated clock widget of current page */
static unsigned char GetNextOutdatedClockWidget(void)
{
  unsigned char i = 0;
  for (; pCurrWidgetList[i].Id != INVALID_ID && i < MAX_WIDGET_NUM; ++i)
  {
    if (pCurrWidgetList[i].Outdated &&
        IS_CLOCK_WIDGET(pCurrWidgetList[i].Layout) &&
        ON_CURRENT_PAGE(pCurrWidgetList[i].Layout))
      return pCurrWidgetList[i].Id;
  }
  return INVALID_ID;
}

unsigned char CurrentIdleScreen(void)
{
  return CurrentPage;
}

static void WriteClockWidget(unsigned char *pBuffer)
{
  unsigned char Id = FACE_ID(*pBuffer);
  unsigned char ClockId = INVALID_ID;
  unsigned char i, k;

  for (i = 0; pCurrWidgetList[i].Id != INVALID_ID && i < MAX_WIDGET_NUM; ++i)
  {
    if (!IS_CLOCK_WIDGET(pCurrWidgetList[i].Layout)) continue;
    
    if (ON_CURRENT_PAGE(pCurrWidgetList[i].Layout))
    {
      if (FACE_ID(pCurrWidgetList[i].Id) == Id)
      {
        unsigned char LayoutType = LAYOUT_TYPE(pCurrWidgetList[i].Layout);
        
        for (k = 0; k < Layout[LayoutType].QuadNum; ++k)
        {
          unsigned int Addr = pCurrWidgetList[i].Buffers[k] * BYTES_PER_QUAD + WGT_BUF_START_ADDR;
          unsigned char *pBuf = pBuffer + k * BYTES_PER_QUAD;
          pBuf[0] = SPI_WRITE;
          pBuf[1] = Addr >> 8;
          pBuf[2] = Addr;
          
          WriteBlockToSram(pBuf, SRAM_HEADER_LEN + BYTES_PER_QUAD);
        }

        pCurrWidgetList[i].Outdated = pdFALSE;
      }
      else if (ClockId == INVALID_ID && pCurrWidgetList[i].Outdated)
        ClockId = pCurrWidgetList[i].Id;
    }
  }
  // update next outdated clock widget of the current page
  if (ClockId == INVALID_ID) CreateAndSendMessage(UpdateDisplayMsg,
    IDLE_MODE | MSG_OPT_NEWUI | MSG_OPT_UPD_INTERNAL | MSG_OPT_UPD_HWGT);
  else CreateAndSendMessage(DrawClockWidgetMsg, ClockId);
}

static unsigned int GetAddr(WidgetHeader_t *pData)
{
  static unsigned char i = 0;
  unsigned int Addr = 0;

  if (pData->Id != pCurrWidgetList[i].Id)
  {
    for (i = 0; pCurrWidgetList[i].Id != INVALID_ID && i < MAX_WIDGET_NUM; ++i)
    {
      if (pData->Id == pCurrWidgetList[i].Id &&
          !IS_CLOCK_WIDGET(pCurrWidgetList[i].Layout)) break;
    }

    if (pCurrWidgetList[i].Id == INVALID_ID || i == MAX_WIDGET_NUM)
    {
      i = 0;
      return Addr;
    }
  }
  
  //find right buffer according to row
  Addr = pCurrWidgetList[i].Buffers[pData->Row / QUAD_ROW_NUM] * BYTES_PER_QUAD +
         (pData->Row % QUAD_ROW_NUM) * BYTES_PER_QUAD_LINE + WGT_BUF_START_ADDR;

  return Addr;
}

static void GetQuadAddr(QuadAddr_t *pAddr)
{
  unsigned char i = 0;
  for (i = 0; pCurrWidgetList[i].Id != INVALID_ID && i < MAX_WIDGET_NUM; ++i)
  {
    if ((pCurrWidgetList[i].Layout & IDLE_PAGE_MASK) >> IDLE_PAGE_SHFT == pAddr->Page)
    {
      unsigned char Type = LAYOUT_TYPE(pCurrWidgetList[i].Layout);
      unsigned char Quad = pCurrWidgetList[i].Layout & QUAD_NO_MASK;
      unsigned char k = 0;
      //PrintS("W:"); PrintH(pCurrWidgetList[i].Id);

      while (k < Layout[Type].QuadNum)
      {
        pAddr->Addr[Quad] = pCurrWidgetList[i].Buffers[k] * BYTES_PER_QUAD + WGT_BUF_START_ADDR;
        pAddr->Layout[Quad] = pCurrWidgetList[i].Layout; //(pCurrWidgetList[i].Layout & LAYOUT_MASK) >> LAYOUT_SHFT;
        Quad += Layout[Type].Step;
        k ++;
      }
    }
  }
}

//#define MSG_OPT_NEWUI             (0x80)
//#define MSG_OPT_SET_PAGE          (0x20)
//#define MSG_OPT_UPD_HWGT          (0x20)
//#define MSG_OPT_UPD_INTERNAL      (0x10)
//#define MSG_OPT_PAGE_NO           (0x0C)
//#define MSG_OPT_TURN_PAGE         (0x0C)
//#define MSG_OPT_PRV_PAGE          (0x08)
//#define MSG_OPT_NXT_PAGE          (0x04)
//#define MODE_MASK                 (0x03)
//#define IDLE_MODE_PAGE_MASK       (0x07)

void UpdateDisplayHandler(tMessage* pMsg)
{
  unsigned char Mode = pMsg->Options & MODE_MASK;
  unsigned char SramBuf[SRAM_HEADER_LEN];
  tLcdData LcdData;
  tMessage Msg;

  if ((pMsg->Options & MSG_OPT_NEWUI) && GetProperty(PROP_PHONE_DRAW_TOP)) //for idle update only
  {
    if (!(pMsg->Options & MSG_OPT_UPD_INTERNAL) && pMsg->Options & MSG_OPT_SET_PAGE)
    { // set current page only
      CurrentPage = (pMsg->Options & MSG_OPT_PAGE_NO) >> SET_PAGE_SHFT;
    }
    else if (pMsg->Options & MSG_OPT_TURN_PAGE)
    { // turn page
      if ((pMsg->Options & MSG_OPT_TURN_PAGE) == MSG_OPT_NXT_PAGE) CurrentPage ++;

      if (CurrentPage == IDLE_PAGE_NUM) CurrentPage = 0;
    }
    
    // not in idle mode idle page
    if (CurrentMode != IDLE_MODE || PageType != PAGE_TYPE_IDLE) return;

    unsigned char OutdatedClkWgt = GetNextOutdatedClockWidget();

    if (!(pMsg->Options & MSG_OPT_UPD_INTERNAL && pMsg->Options & MSG_OPT_UPD_HWGT) &&
        OutdatedClkWgt != INVALID_ID)
    { //ask for drawing clock widget from external
      PrintF("- UpdDsp OtdClk:0x%02X", OutdatedClkWgt);
      SendMessage(&Msg, DrawClockWidgetMsg, OutdatedClkWgt);
      return; // will get back internal upddisp when updhomewgt done
    }
    
//    PrintC('U');
    xSemaphoreTake(SramMutex, portMAX_DELAY);

    QuadAddr_t QuadAddr;
    QuadAddr.Page = CurrentPage;

    // clear QuadAddr
    unsigned char i;
    for (i = 0; i < QUAD_NUM; ++i)
    {
      QuadAddr.Addr[i] = 0;
      QuadAddr.Layout[i] = 0;
    }
    GetQuadAddr(&QuadAddr);
    
    i = 0;
    while (BufTag & (1 << i)) i ++; // find out first empty buffer
    unsigned int Addr = i * BYTES_PER_QUAD + WGT_BUF_START_ADDR;
    
    for (i = 0; i < QUAD_NUM; ++i)
    {
     if (QuadAddr.Addr[i] == 0) QuadAddr.Addr[i] = Addr;
    }

    unsigned char Row = 0;
    i = 0; // 0 for upper Quads, 1 for lower Quads

    while (Row < LCD_ROW_NUM)
    {
      unsigned char k = 1; // 0 for left Quads, 1 for right Quads
      do
      {
        Addr = QuadAddr.Addr[i+k] + (Row % HALF_SCREEN_ROWS) * BYTES_PER_QUAD_LINE;
        
        SramBuf[0] = MSG_OPT_NEWUI; // tell ReadBlock to use shorter line
        SramBuf[1] = Addr >> 8;
        SramBuf[2] = Addr;

        ReadBlock(SramBuf, (unsigned char *)&LcdData + BYTES_PER_QUAD_LINE * k);

        unsigned char c; // Column byte number
        
        if (QuadAddr.Layout[i+k] & INVERT_BIT)
        { // Invert pixel
          for (c = 0; c < BYTES_PER_QUAD_LINE; ++c)
            LcdData.pLine[c + BYTES_PER_QUAD_LINE * k] = ~LcdData.pLine[c + BYTES_PER_QUAD_LINE * k];
        }

        if (GetProperty(PROP_WIDGET_GRID))
        {
          // Rule 1: outer upper and lower horizontal boarders
//          if (Row == 0 || Row == LCD_ROW_NUM - 1)
//          {
//            for (c = 0; c < BYTES_PER_QUAD_LINE; ++c)
//              LcdData.pLine[c + BYTES_PER_QUAD_LINE * k] = BOARDER_PATTERN_ROW;
//          }
//          else
          {
            unsigned char LayoutType = LAYOUT_TYPE(QuadAddr.Layout[i+k]);

            if (Row % BOARDER_PATTERN_COL == 1 || Row % BOARDER_PATTERN_COL == 2)
            {// black dots
              // Rule 2: left and right side vertical boarders
//              if (k) LcdData.pLine[BYTES_PER_LINE - 1] |= 0x80; // right side boarder
//              else  LcdData.pLine[0] |= 0x01; // left side boarder

             // Rule 3 inner vertical boarders
             if (LayoutType == 0 || LayoutType == 2)
             {
              if (k) LcdData.pLine[BYTES_PER_QUAD_LINE] |= 0x01; // right inner boarder
              else  LcdData.pLine[BYTES_PER_QUAD_LINE - 1] |= 0x80; // left inner boarder
             }
            }
            else
            {// white dots
              // Rule 2: left and right side vertical boarders
//              if (k) LcdData.pLine[BYTES_PER_LINE - 1] &= 0x7F; // right side boarder
//              else  LcdData.pLine[0] &= 0xFE; // left side boarder

             // Rule 3 inner vertical boarders
             if (LayoutType == 0 || LayoutType == 2)
             {
              if (k) LcdData.pLine[BYTES_PER_QUAD_LINE] &= 0xFE; // right inner boarder
              else  LcdData.pLine[BYTES_PER_QUAD_LINE - 1] &= 0x7F; // left inner boarder
             }
            }

            // Rule 4: inner horizontal boarders
            if (LayoutType == 0 || LayoutType == 1)
            {
              if (Row == HALF_SCREEN_ROWS - 1 || Row == HALF_SCREEN_ROWS)
              {
                for (c = 0; c < BYTES_PER_QUAD_LINE; ++c)
                  LcdData.pLine[c + BYTES_PER_QUAD_LINE * k] = BOARDER_PATTERN_ROW;
              }
            }
          }
        }
      }  while (k--);

      LcdData.RowNumber = Row ++; // Lcd row number starts from 1
      WriteLcdHandler(&LcdData);
      if (Row == HALF_SCREEN_ROWS) i += 2;
    }
    xSemaphoreGive(SramMutex);
  }
  else
  {
    // In watch_draw_top case, if mode change back to idle from other modes,
    // redraw date&time. In any case of mode change, notify LcdDisplay to
    // switch button definition for the mode.
    // for external msg, NOTIF > APP > IDLE > MUSIC
    if (!(pMsg->Options & MSG_OPT_UPD_INTERNAL))
    {
      signed char Result = ComparePriority(Mode);
      PrintF("- UpdDsp Priority:%d", Result);

      if (Result > 0) SendMessage(&Msg, ChangeModeMsg, Mode);
      else if (Result < 0) return;
      else if (CurrentMode != IDLE_MODE) ResetModeTimer();
      else if (PageType != PAGE_TYPE_IDLE) return;
    }
    PrintF("- UpdDsp PgTp:%d", PageType);
    
    // determine starting line
    unsigned char StartRow = (Mode == IDLE_MODE && !GetProperty(PROP_PHONE_DRAW_TOP)) ?
                            WATCH_DRAW_SCREEN_ROW_NUM : 0;
    unsigned char RowNum = LCD_ROW_NUM - StartRow;

    if (Mode == CurrentMode && pMsg->Length)
    {
      Rect_t *pRect = (Rect_t *)pMsg->pBuffer;
      if (pRect->StartRow < LCD_ROW_NUM) StartRow = pRect->StartRow;
      if (pRect->RowNum && pRect->RowNum + StartRow <= LCD_ROW_NUM) RowNum = pRect->RowNum;
    }

    /* now calculate the absolute address */
    unsigned int Addr = (pMsg->Options & IDLE_MODE_PAGE_MASK) * BYTES_PER_SCREEN +
                        StartRow * BYTES_PER_LINE + MODE_BUF_START_ADDR;

    while (RowNum --)
    {
      /* one buffer is used for writing and another is used for reading
       * the incoming message can't be used because it doesn't have a buffer
       */
      SramBuf[0] = 0; // tell ReadBlock to read 12 bytes
      SramBuf[1] = (unsigned char)(Addr >> 8);
      SramBuf[2] = (unsigned char) Addr;

      /*
       * The tLcdData accounts for the
       * 3+1 spots to starting location of data from dma read
       * (room for bytes read in when cmd and address are sent)
       */
      ReadBlock(SramBuf, (unsigned char *)&LcdData);

      /* now add the row number */
      LcdData.RowNumber = StartRow ++;
      WriteLcdHandler(&LcdData);
      Addr += BYTES_PER_LINE;
    }
  }
  /* now that the screen has been drawn put the LCD into a lower power mode */
  PutLcdIntoStaticMode();
}

static signed char ComparePriority(unsigned char Mode)
{
  if (Mode == CurrentMode) return 0;
  
  unsigned char i = 0;
  for (; i < MODE_NUM; ++i) if (ModePriority[i] == Mode) break;
  for (++i; i < MODE_NUM; ++i) if (ModePriority[i] == CurrentMode) return 1;
  return -1;
}

/* use DMA to write a block of data to the serial ram */
static void WriteBlockToSram(const unsigned char* pData, unsigned int Size)
{
  EnableSmClkUser(SERIAL_RAM_USER);
  DmaBusy = 1;
  SRAM_CSN_ASSERT();

  /* USCIA0 TXIFG is the DMA trigger */
  DMACTL0 = DMA0TSEL_17;

  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) pData);
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &UCA0TXBUF);

  DMA0SZ = Size;

  /*
   * single transfer, increment source address, source byte and dest byte,
   * level sensitive, enable interrupt, clear interrupt flag
   */
  DMA0CTL = DMADT_0 + DMASRCINCR_3 + DMASBDB + DMALEVEL + DMAIE;

  /* start the transfer */
  DMA0CTL |= DMAEN;

  while(DmaBusy);
  SRAM_CSN_DEASSERT();
  DisableSmClkUser(SERIAL_RAM_USER);
}

#if __IAR_SYSTEMS_ICC__
static void WriteData20BlockToSram(unsigned char __data20* pData,unsigned int Size)
{
  DmaBusy = 1;
  EnableSmClkUser(SERIAL_RAM_USER);
  SRAM_CSN_ASSERT();

  /* USCIA0 TXIFG is the DMA trigger */
  DMACTL0 = DMA0TSEL_17;

  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) pData);
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &UCA0TXBUF);

  DMA0SZ = Size;

  /*
   * single transfer, increment source address, source byte and dest byte,
   * level sensitive, enable interrupt, clear interrupt flag
   */
  DMA0CTL = DMADT_0 + DMASRCINCR_3 + DMASBDB + DMALEVEL + DMAIE;

  /* start the transfer */
  DMA0CTL |= DMAEN;

  while(DmaBusy);
  SRAM_CSN_DEASSERT();
  DisableSmClkUser(SERIAL_RAM_USER);
}
#endif

static void SetupCycle(unsigned int Address,unsigned char CycleType)
{
  SRAM_CSN_ASSERT();
  UCA0TXBUF = CycleType;
  while (!(UCA0IFG&UCTXIFG));
  while (!(UCA0IFG&UCRXIFG));
  ReadData = UCA0RXBUF;

  /* write 16 bit address (only 13 bits are used) */
  UCA0TXBUF = (unsigned char)(Address >> 8);
  while (!(UCA0IFG&UCTXIFG));
  while (!(UCA0IFG&UCRXIFG));
  ReadData = UCA0RXBUF;

  /*
   * wait until read is done (transmit must be done)
   * then clear read flag so it is ready for the DMA
  */
  UCA0TXBUF = (unsigned char)Address;
  while (!(UCA0IFG&UCTXIFG));
  while (!(UCA0IFG&UCRXIFG));
  ReadData = UCA0RXBUF;
}

static void ReadBlock(unsigned char* pWriteData,unsigned char* pReadData)
{
  DmaBusy = 1;
  SRAM_CSN_ASSERT();

  unsigned char DataLen = (*pWriteData == MSG_OPT_NEWUI) ?
      SRAM_HEADER_LEN + BYTES_PER_QUAD_LINE + 1 : SRAM_HEADER_LEN + BYTES_PER_LINE + 1;
  *pWriteData = SPI_READ;

  /*
   * SPI has to write bytes to receive bytes so
   *
   * two DMA channels are used
   *
   * read requires 16 total bytes because the shift in of the read data
   * lags the transmit data by one byte
   */

  /* USCIA0 TXIFG is the DMA trigger for DMA0 and RXIFG is the DMA trigger
   * for dma1 (DMACTL0 controls both)
   */
  DMACTL0 = DMA1TSEL_16 | DMA0TSEL_17;
  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) pWriteData);
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &UCA0TXBUF);
  DMA0SZ = DataLen;
  /* don't enable interrupt for transmit dma done(channel 0)
   * increment the source address because the message contains the address
   * the other bytes are don't care
   */
  DMA0CTL = DMADT_0 + DMASRCINCR_3 + DMASBDB + DMALEVEL;

  /* receive data is source for dma 1 */
  __data16_write_addr((unsigned short) &DMA1SA,(unsigned long) &UCA0RXBUF);
  __data16_write_addr((unsigned short) &DMA1DA,(unsigned long) pReadData);
  DMA1SZ = DataLen;
  /* increment destination address */
  DMA1CTL = DMADT_0 + DMADSTINCR_3 + DMASBDB + DMALEVEL + DMAIE;

  /* start the transfer */
  DMA1CTL |= DMAEN;
  DMA0CTL |= DMAEN;

  while(DmaBusy);
  SRAM_CSN_DEASSERT();
}

static void ClearSram(unsigned int Address, unsigned char Data, unsigned int Size)
{
  DmaBusy = 1;
  EnableSmClkUser(SERIAL_RAM_USER);
  SetupCycle(Address, SPI_WRITE);

  /* USCIA0 TXIFG is the DMA trigger */
  DMACTL0 = DMA0TSEL_17;

  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &Data);
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &UCA0TXBUF);

  DMA0SZ = Size;

  /*
   * single transfer, DON'T increment source address, source byte and dest byte,
   * level sensitive, enable interrupt, clear interrupt flag
   */
  DMA0CTL = DMADT_0 + DMASBDB + DMALEVEL + DMAIE;

  /* start the transfer */
  DMA0CTL |= DMAEN;

  while(DmaBusy);
  SRAM_CSN_DEASSERT();
  DisableSmClkUser(SERIAL_RAM_USER);
}

/* Load a template from flash into a draw buffer (ram)
 * This can be used by the phone or the watch application to save drawing time
 */
void LoadTemplateHandler(tMessage* pMsg)
{
  unsigned int Addr = (pMsg->Options & MODE_MASK) * BYTES_PER_SCREEN + MODE_BUF_START_ADDR;
  /*
   * templates don't have extra space in them for additional 3 bytes of
   * cmd and address
   */

  if (pMsg->pBuffer == NULL)
  { // internal usage
    SetupCycle(Addr, SPI_WRITE);
    WriteBlockToSram(pTemplate[pMsg->Options >> 4], BYTES_PER_SCREEN);
  }
  else if (*pMsg->pBuffer <= 1)
  {
    /* clear or fill the screen */
    ClearSram(Addr, *pMsg->pBuffer ? 0xFF : 0x00, BYTES_PER_SCREEN);
  }
  else
  {
#if __IAR_SYSTEMS_ICC__
    /* template zero is reserved for simple patterns */
    SetupCycle(Addr, SPI_WRITE);
    WriteData20BlockToSram(
      (unsigned char __data20*)&pWatchFace[*pMsg->pBuffer - TEMPLATE_1][0],
      BYTES_PER_SCREEN);
    
    PrintF("-Template:%d", *pMsg->pBuffer);
#endif
  }
}

static void LoadBuffer(unsigned char i, const unsigned char *pTemp)
{
  SetupCycle(i * BYTES_PER_QUAD + WGT_BUF_START_ADDR, SPI_WRITE);
  WriteBlockToSram(pTemp, BYTES_PER_QUAD);
}

/* configure the MSP430 SPI peripheral */
void SerialRamInit(void)
{
  /* assert reset when configuring */
  UCA0CTL1 = UCSWRST;

  EnableSmClkUser(SERIAL_RAM_USER);

  SRAM_SCLK_PSEL |= SRAM_SCLK_PIN;
  SRAM_SOMI_PSEL |= SRAM_SOMI_PIN;
  SRAM_SIMO_PSEL |= SRAM_SIMO_PIN;

  /* 3 pin SPI master, MSB first, clock inactive when low, phase is 1 */
  UCA0CTL0 |= UCMST + UCMSB + UCCKPH + UCSYNC;

  UCA0CTL1 |= UCSSEL__SMCLK;

  /* spi clock of 8.39 MHz (chip can run at 16 MHz max)*/
  UCA0BR0 = 0x02;
  UCA0BR1 = 0x00;

  /* release reset and wait for SPI to initialize */
  UCA0CTL1 &= ~UCSWRST;

  vTaskDelay(SPI_INIT_DELAY_IN_MS);

  /*
   * Read the status register
   */
  SRAM_CSN_ASSERT();

  while (!(UCA0IFG&UCTXIFG));

  /* writing automatically clears flag */
  UCA0TXBUF = SPI_RDSR;
  while (!(UCA0IFG&UCTXIFG));
  UCA0TXBUF = DummyData;
  while (!(UCA0IFG&UCTXIFG));
  WAIT_FOR_SRAM_SPI_SHIFT_COMPLETE();
  ReadData = UCA0RXBUF;

  unsigned char FinalSrValue = DEFAULT_SR_VALUE;
  unsigned char DefaultSrValue = FINAL_SR_VALUE;

  if ( GetBoardConfiguration() >= 2 )
  {
    DefaultSrValue = DEFAULT_SR_VALUE_256;
    FinalSrValue = FINAL_SR_VALUE_256;
  }

  /* make sure correct value is read from the part */
  if (ReadData != DefaultSrValue && ReadData != FinalSrValue) PrintS("# SRAM Init1");
  SRAM_CSN_DEASSERT();

  /*
   * put the part into sequential mode
   */
  SRAM_CSN_ASSERT();
  UCA0TXBUF = SPI_WRSR;
  while (!(UCA0IFG&UCTXIFG));
  UCA0TXBUF = SEQUENTIAL_MODE_COMMAND;
  while (!(UCA0IFG&UCTXIFG));
  WAIT_FOR_SRAM_SPI_SHIFT_COMPLETE();
  SRAM_CSN_DEASSERT();

  /**/
  SRAM_CSN_ASSERT();
  UCA0TXBUF = SPI_RDSR;
  while (!(UCA0IFG&UCTXIFG));

  UCA0TXBUF = DummyData;
  while (!(UCA0IFG&UCTXIFG));
  WAIT_FOR_SRAM_SPI_SHIFT_COMPLETE();
  ReadData = UCA0RXBUF;

  /* make sure correct value is read from the part */
  if (ReadData != FinalSrValue) PrintS("# SRAM Init2");
  SRAM_CSN_DEASSERT();

  /* now use the DMA to clear the serial ram */
  ClearSram(MODE_BUF_START_ADDR, DummyData, MODE_BUFFER_SIZE);

  unsigned char i;
  for (i = 0; i < MAX_WIDGET_NUM + MAX_WIDGET_NUM; ++i)
    Widget[i].Id = INVALID_ID;

  // load 16 buffers with empty widget template
  for (i = 0; i < MAX_WIDGET_NUM; ++i) LoadBuffer(i, pWidgetTemplate[TMPL_WGT_EMPTY]);
  
  SramMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(SramMutex);
  DisableSmClkUser(SERIAL_RAM_USER);
}

/* Serial RAM controller uses two dma channels
 * LCD driver task uses one dma channel
 */
#ifndef __IAR_SYSTEMS_ICC__
#pragma CODE_SECTION(DMA_ISR,".text:_isr");
#endif

#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  LAST_CRITICAL_CODE(CC_DMA_ISR);
  CODE_START(dmaISR);

  /* 0 is no interrupt and remainder are channels 0-7 */
  switch(__even_in_range(DMAIV,16))
  {
  case 0: break;
  case 2: DmaBusy = 0; break;
  case 4: DmaBusy = 0; break;
  case 6: LcdDmaIsr(); break;
  default: break;
  }

  CODE_END(dmaISR);
}
