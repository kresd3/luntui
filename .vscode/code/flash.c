#include "zf_common_headfile.h"
#include "flash.h"

static flash_data_union flash_buffer_backup[FLASH_PAGE_LENGTH];

static void flash_buffer_backup_save(void)
{
    for(uint16 i = 0; i < FLASH_PAGE_LENGTH; i++)
    {
        flash_buffer_backup[i].uint32_type = flash_union_buffer[i].uint32_type;
    }
}

static void flash_buffer_backup_restore(void)
{
    for(uint16 i = 0; i < FLASH_PAGE_LENGTH; i++)
    {
        flash_union_buffer[i].uint32_type = flash_buffer_backup[i].uint32_type;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入自转事件组
// 参数说明     void
// 返回参数     void
// 使用示例     flash_Turn_Write();
// 备注信息     将自转事件组、路径总点数和自转事件数量写入 Turn_Flash_Page，写入前会备份当前 flash 缓冲区
//-------------------------------------------------------------------------------------------------------------------
void flash_Turn_Write(void)
{
    flash_buffer_backup_save();
    flash_buffer_clear();

    for(uint16 i = 0; i < N.Turn_Save_index; i++)
    {
        uint16 base = Turn_DataStart + i * Turn_ItemSize;

        if(i >= Turn_MaxSize) break;
        if(base + Turn_ItemSize - 1 >= MaxSize) break;

        flash_union_buffer[base].uint32_type = Turn_read[i].start_index;
        flash_union_buffer[base + 1].int32_type = Turn_read[i].direction;
    }

    if(flash_check(0, Turn_Flash_Page))
        flash_erase_page(0, Turn_Flash_Page);

    flash_write_page_from_buffer(0, Turn_Flash_Page, FLASH_PAGE_LENGTH);

    flash_buffer_backup_restore();
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取自转事件组
// 参数说明     void
// 返回参数     void
// 使用示例     flash_Turn_Read();
// 备注信息     从 Turn_Flash_Page 读取自转事件组，并校验事件数量、路径计值范围和自转方向
//-------------------------------------------------------------------------------------------------------------------
void flash_Turn_Read(void)
{
    flash_buffer_clear();
    memset(Turn_read, 0, sizeof(Turn_read));

    if(flash_check(0, Turn_Flash_Page))
    {
        flash_read_page_to_buffer(0, Turn_Flash_Page, FLASH_PAGE_LENGTH);
    }

    uint16 valid_count = 0;

    for(uint16 i = 0; i < N.Turn_Save_index && i < Turn_MaxSize; i++)
    {
        uint16 base = Turn_DataStart + i * Turn_ItemSize;

        if(base + Turn_ItemSize - 1 >= MaxSize) break;

        uint16 start_index = (uint16)flash_union_buffer[base].uint32_type;
        int8 direction = (int8)flash_union_buffer[base + 1].int32_type;

        if(start_index >= N.Save_index) break;
        if(direction != Nag_Turn_Left && direction != Nag_Turn_Right) break;

        if(valid_count > 0 &&
           start_index <= Turn_read[valid_count - 1].start_index)
        {
            break;
        }

        Turn_read[valid_count].start_index = start_index;
        Turn_read[valid_count].direction = direction;

        valid_count++;
    }

    N.Turn_Save_index = valid_count;
    N.Turn_Run_index = 0;
    N.Turn_Run_State = Nag_Turn_State_Path;
    N.Turn_Target_Angle = 0;
    N.Turn_Angle_All = 0;
    N.Turn_Last_Yaw = 0;

    flash_buffer_clear();
}


void flash_Nag_Write(){
   
    if(flash_check(0, N.Flash_page_index))flash_erase_page(0, N.Flash_page_index);                  
                       
    flash_write_page_from_buffer(0,N.Flash_page_index,FLASH_PAGE_LENGTH);
    if(N.End_f == 1)
    {
        if(flash_check(0, Nag_End_Page))     flash_erase_page(0, Nag_End_Page);

        flash_union_buffer[MaxSize + 2].uint32_type = N.Save_index;
        flash_union_buffer[MaxSize + 3].uint32_type = N.Turn_Save_index;

        flash_write_page_from_buffer(0, Nag_End_Page, FLASH_PAGE_LENGTH);
    }
      
    flash_buffer_clear();
   
}

void flash_Nag_Read(){
    flash_buffer_clear();
    static uint8 Index_R_f=0;

    if( 0 == Index_R_f)
    {
        flash_read_page_to_buffer(0, Nag_End_Page, FLASH_PAGE_LENGTH);
        N.Save_index = flash_union_buffer[MaxSize + 2].uint32_type;
        N.Turn_Save_index = flash_union_buffer[MaxSize + 3].uint32_type;
        if(N.Save_index > Read_MaxSize) N.Save_index = 0;
        if(N.Turn_Save_index > Turn_MaxSize) N.Turn_Save_index = 0;    
        Index_R_f=1;
        flash_buffer_clear();
    }
    if(flash_check(0, N.Flash_page_index))
    {        
        flash_read_page_to_buffer(0, N.Flash_page_index,FLASH_PAGE_LENGTH);
    }
}

