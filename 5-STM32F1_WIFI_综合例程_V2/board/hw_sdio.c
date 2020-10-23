/******************************************************************************
  * @file    hw_sdio.h
  * @author  Yu-ZhongJun
  * @version V0.0.1
  * @date    2018-7-31
  * @brief   sdio��Դ�ļ�
******************************************************************************/
#include "hw_sdio.h"

sdio_func_t hw_sdio_func[SDIO_FUNC_MAX];
sdio_core_t hw_sdio_core;
psdio_core_t phw_sdio_core = &hw_sdio_core;

/* ���������� */
static uint8_t hw_sdio_cmd3(uint32_t para,uint32_t *resp);
static uint8_t hw_sdio_cmd5(uint32_t para,uint32_t *resp,uint32_t retry_max);
static uint8_t hw_sdio_cmd7(uint32_t para,uint32_t *resp);
static uint8_t hw_sdio_cmd53_read(uint8_t func_num,uint32_t address, uint8_t incr_addr, uint8_t *buf,uint32_t size,uint16_t cur_blk_size);
static uint8_t hw_sdio_cmd53_write(uint8_t func_num,uint32_t address, uint8_t incr_addr, uint8_t *buf,uint32_t size,uint16_t cur_blk_size);
static uint8_t hw_sdio_core_init(void);
static uint8_t hw_sdio_check_err(void);
static uint8_t hw_sdio_parse_r4(uint32_t r4);
static uint8_t hw_sdio_parse_r6(uint32_t r6,uint32_t *rca);
static uint8_t hw_sdio_cis_read_parse(uint8_t func_num,uint32_t cis_ptr);
static uint8_t hw_sdio_set_dblocksize(uint32_t *struct_dblocksize,uint32_t block_size);

/******************************************************************************
 *	������:	hw_sdio_init
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		SDIO init
 				pin�ŷ���
 				PC8->SDIO D0 PC9->SDIO D1 PC10->SDIO D2 PC11->SDIO D3
 				PC12->SDIO CLK
 				PD2->SDIO CMD
******************************************************************************/
uint8_t hw_sdio_init()
{
    uint32_t rca;
    uint8_t func_index;
    uint32_t cmd3_para;
    uint32_t cmd3_resp;
    uint32_t cmd5_para;
    uint32_t cmd5_resp;
    uint32_t cmd7_para;
    uint32_t cmd7_resp;

    GPIO_InitTypeDef GPIO_InitStructure;
    SDIO_InitTypeDef SDIO_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;

    HW_ENTER();
    hw_chip_reset();
    hw_sdio_core_init();
    /* ʹ��GPIO C/D��RCCʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    /* ʹ��SDIO RCCʱ��*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);
    /* ʹ��DMA2ʱ��,SDIO��DMA��DMA2 CHANNEL 4 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /* �������츴�� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* SDIO��ʼ����CLK:400KHZ,���ݿ��:1 bus */
    SDIO_DeInit();
    /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_CLK_400KHZ;
    SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;// SDIO_ClockEdge_Falling; // SDIO_ClockEdge_Rising ;
    SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
    SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    //SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Enable;
    SDIO_Init(&SDIO_InitStructure);

    /* ����ΪSDIO I/Oģʽ */
    SDIO_SetSDIOOperation(ENABLE);
    /* Set Power State to ON */
    SDIO_SetPowerState(SDIO_PowerState_ON);
    /* Enable SDIO Clock */
    SDIO_ClockCmd(ENABLE);

    /* ����SDIO�жϣ���ռ���ȼ�Ϊ3����Ӧ���ȼ�Ϊ4 */
#if 0
    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = WIFI_PREE_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = WIFI_SUB_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    SDIO_ITConfig(SDIO_IT_CCRCFAIL |SDIO_IT_DCRCFAIL | SDIO_IT_CTIMEOUT |\
                  SDIO_IT_DTIMEOUT | SDIO_IT_TXUNDERR |SDIO_IT_RXOVERR | \
                  SDIO_IT_STBITERR|SDIO_IT_SDIOIT, ENABLE);
#endif
    /* ����SDIO��DMA����λ��������Ҫ�������CMD53��ÿ�ο�������ô��DMA lockס */
    SDIO_DMACmd(ENABLE);

    /* ����cmd5 */
    cmd5_para = 0;
    if(hw_sdio_cmd5(cmd5_para,&cmd5_resp,SDIO_RETRY_MAX))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_CMD5_FAIL;
    }

    /* ocr 3.2V~3.4V*/
    cmd5_para = 0x300000;
    if(hw_sdio_cmd5(cmd5_para,&cmd5_resp,SDIO_RETRY_MAX))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_CMD5_FAIL;
    }

    /* ����R4 */
    hw_sdio_parse_r4(cmd5_resp);

    /* ����cmd3��ȡ��ַ */
    cmd3_para = 0;
    if(hw_sdio_cmd3(cmd3_para,&cmd3_resp))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_CMD3_FAIL;
    }

    hw_sdio_parse_r6(cmd3_resp,&rca);

    /* ����cmd7ѡ��ַ */
    cmd7_para = rca << 16;
    if(hw_sdio_cmd7(cmd7_para,&cmd7_resp))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_CMD7_FAIL;
    }

    /* ��ȡCCCR�汾��SDIO�汾 */
    hw_sdio_get_cccr_version(&phw_sdio_core->cccr_version);
    hw_sdio_get_sdio_version(&phw_sdio_core->sdio_version);

    /* �л���4 bus width,�л�24M clk */
    hw_sdio_set_bus_width(SDIO_BUS_WIDTH_4);
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_CLK_24MHZ;
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
    SDIO_Init(&SDIO_InitStructure);


    /* ��ȡÿ��func��CISָ�벢�ҽ��� */
    for(func_index = 0; func_index < phw_sdio_core-> func_total_num; func_index++)
    {
        uint32_t cis_ptr;
        hw_sdio_get_cis_ptr(func_index,&cis_ptr);
        hw_sdio_cis_read_parse(func_index,cis_ptr);
    }

    /* enable Func */
    for(func_index = SDIO_FUNC_1; func_index < phw_sdio_core-> func_total_num; func_index++)
    {
        hw_sdio_enable_func(func_index);
    }

    /* ʹ���ж� */
    hw_sdio_enable_mgr_int();
    for(func_index = SDIO_FUNC_1; func_index < phw_sdio_core-> func_total_num; func_index++)
    {
        hw_sdio_enable_func_int(func_index);
    }

    /* ����block size */
    for(func_index = SDIO_FUNC_1; func_index < phw_sdio_core-> func_total_num; func_index++)
    {
        hw_sdio_set_blk_size(func_index,SDIO_DEFAULT_BLK_SIZE);
    }
    HW_LEAVE();

    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_get_cccr_version
 * ����:  		cccr_version(OUT)		-->CCCR�汾
 * ����ֵ: 	����ִ�н��
 * ����:		ͨ����ȡCCCR�Ĵ���������CCCR�Ĵ����汾
 				CCCR�汾�ļĴ�����ַΪ0x0
 				��ʽΪ:
 				|-7-|-6-|-5-|-4-|-3-|-2-|-1-|-0-|
 				|--SDIO version-| CCCR version  |
 				Value CCCR/FBR Format Version
				00h CCCR/FBR defined in SDIO Version 1.00
				01h CCCR/FBR defined in SDIO Version 1.10
				02h CCCR/FBR defined in SDIO Version 2.00
				03h CCCR/FBR defined in SDIO Version 3.00
				04h-0Fh Reserved for Future Use
******************************************************************************/
uint8_t hw_sdio_get_cccr_version(uint8_t *cccr_version)
{
    uint8_t version;
    HW_ENTER();
    if(!cccr_version)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* CMD52����CCCR0��ֵ */
    if(hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_SDIO_VERSION,0,&version))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_GET_VER_FAIL;
    }

    *cccr_version = phw_sdio_core->cccr_version = version & 0xf;

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_get_sdio_version
 * ����:  		sdio_version(OUT)		-->SDIO�汾
 * ����ֵ: 	����ִ�н��
 * ����:		ͨ����ȡCCCR�Ĵ���������SDIO�汾
 				SDIO�汾�ļĴ�����ַΪ0x0
 				��ʽΪ:
 				|-7-|-6-|-5-|-4-|-3-|-2-|-1-|-0-|
 				|--SDIO version-| CCCR version  |
 				Value SDIO Specification
				00h SDIO Specification Version 1.00
				01h SDIO Specification Version 1.10
				02h SDIO Specification Version 1.20 (unreleased)
				03h SDIO Specification Version 2.00
				04h SDIO Specification Version 3.00
				05h-0Fh Reserved for Future Use
******************************************************************************/
uint8_t hw_sdio_get_sdio_version(uint8_t *sdio_version)
{
    uint8_t version;
    HW_ENTER();
    if(!sdio_version)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* CMD52����CCCR0��ֵ */
    if(hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_SDIO_VERSION,0,&version))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_GET_VER_FAIL;
    }

    *sdio_version = phw_sdio_core->sdio_version = (version>>4) & 0xf;
    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_enable_func
 * ����:  		func_num(IN)		-->func num
 * ����ֵ: 	����ִ�н��
 * ����:		����func numʹ���ض���func
******************************************************************************/
uint8_t hw_sdio_enable_func(uint8_t func_num)
{
    uint8_t enable;
    uint8_t ready = 0;
    HW_ENTER();
    if(func_num > SDIO_FUNC_MAX)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* CMD52����CCCR2��ֵ */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_IO_ENABLE,0,&enable))
    {
        /* ����������ƻ�ԭ�е�ֵ */
        enable |= (1<<func_num);
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_ENABLE_FUNC_FAIL;
    }

    /* ����д��CCCR2 */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_IO_ENABLE,enable,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_ENABLE_FUNC_FAIL;
    }

    /* �ȴ��ض���func ready */
    while(!(ready & (1<<func_num)))
    {
        hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_IO_READY,0,&ready);
    }

    /* ����func��״̬ */
    if((phw_sdio_core->func)[func_num])
    {
        (phw_sdio_core->func)[func_num]->func_status = FUNC_ENABLE;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_disable_func
 * ����:  		func_num(IN)		-->func num
 * ����ֵ: 	����ִ�н��
 * ����:		����func numʧ���ض���func
******************************************************************************/
uint8_t hw_sdio_disable_func(uint8_t func_num)
{
    uint8_t enable;
    HW_ENTER();
    if(func_num > SDIO_FUNC_MAX)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }
    /* CMD52����CCCR2��ֵ */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_IO_ENABLE,0,&enable))
    {
        /* �뷴���������ƻ�ԭ�е�ֵ */
        enable &= ~(1<<func_num);

    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_DISABLE_FUNC_FAIL;
    }

    /* ����д��CCCR2 */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_IO_ENABLE,enable,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_DISABLE_FUNC_FAIL;
    }

    /* ����func��״̬ */
    if((phw_sdio_core->func)[func_num])
    {
        (phw_sdio_core->func)[func_num]->func_status = FUNC_DISABLE;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}


/******************************************************************************
 *	������:	hw_sdio_enable_func_int
 * ����:  		func_num(IN)		-->func num
 * ����ֵ: 	����ִ�н��
 * ����:		����func numʹ���ض���func���ж�
******************************************************************************/
uint8_t hw_sdio_enable_func_int(uint8_t func_num)
{
    uint8_t enable;
    HW_ENTER();
    if(func_num > SDIO_FUNC_MAX)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* CMD52����CCCR4��ֵ */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,0,&enable))
    {
        /* ����������ƻ�ԭ�е�ֵ */
        enable |= (1<<func_num);
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_ENABLE_FUNC_INT_FAIL;
    }

    /* ����д��CCCR4 */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,enable,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_ENABLE_FUNC_INT_FAIL;
    }

    /* ����func���ж�״̬ */
    if((phw_sdio_core->func)[func_num])
    {
        (phw_sdio_core->func)[func_num]->func_int_status = FUNC_INT_ENABLE;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_disable_func_int
 * ����:  		func_num(IN)		-->func num
 * ����ֵ: 	����ִ�н��
 * ����:		����func numʧ���ض���func���ж�
******************************************************************************/
uint8_t hw_sdio_disable_func_int(uint8_t func_num)
{
    uint8_t enable;
    HW_ENTER();
    if(func_num > SDIO_FUNC_MAX)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* CMD52����CCCR4��ֵ */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,0,&enable))
    {
        /* �뷴���������ƻ�ԭ�е�ֵ */
        enable &= ~(1<<func_num);
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_DISABLE_FUNC_INT_FAIL;
    }

    /* ����д��CCCR4 */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,enable,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_DISABLE_FUNC_INT_FAIL;
    }

    /* ����func���ж�״̬ */
    if((phw_sdio_core->func)[func_num])
    {
        (phw_sdio_core->func)[func_num]->func_int_status = FUNC_INT_DISABLE;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_enable_mgr_int
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		����func numʹ��CCCR int�ܿ���
******************************************************************************/
uint8_t hw_sdio_enable_mgr_int()
{
    uint8_t enable;
    HW_ENTER();

    /* ����CCCR4��ֵ */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,0,&enable))
    {
        /* �ж��ܿ�����CCCR4��bit 0,���Ի���1 */
        enable |= 0x1;
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_ENABLE_MGR_INT_FAIL;
    }

    /* ����д��ȥ */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,enable,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_ENABLE_MGR_INT_FAIL;
    }

    /* ����INT manager��״̬ */
    phw_sdio_core->sdio_int_mgr = FUNC_INT_ENABLE;

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_disable_mgr_int
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		����func numʧ��CCCR int�ܿ���
******************************************************************************/
uint8_t hw_sdio_disable_mgr_int()
{
    uint8_t enable;
    HW_ENTER();

    /* ����CCCR4��ֵ */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,0,&enable))
    {
        /* �ж��ܿ�����CCCR4��bit 0,�����뷴��1 */
        enable &= ~0x1;
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_DISABLE_MGR_INT_FAIL;
    }

    /* ����д��ȥ */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_INT_ENABLE,enable,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_DISABLE_MGR_INT_FAIL;
    }

    /* ����INT manager��״̬ */
    phw_sdio_core->sdio_int_mgr = FUNC_INT_DISABLE;

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_get_int_pending
 * ����:  		int_pending(OUT)		-->�ж�pending��ֵ
 * ����ֵ: 	����ִ�н��
 * ����:		��ȡ�ж�pending��ֵ
******************************************************************************/
uint8_t hw_sdio_get_int_pending(uint8_t *int_pending)
{
    HW_ENTER();
    if(!int_pending)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* �����ж�pending��ֵ */
    if(hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_INT_PENDING,0,int_pending))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_GET_INT_PEND_FAIL;
    }
    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_set_func_abort
 * ����:  		func_num(IN)		-->func���
 * ����ֵ: 	����ִ�н��
 * ����:		����ĳһ��func abort
******************************************************************************/
uint8_t hw_sdio_set_func_abort(uint8_t func_num)
{
    uint8_t abort;
    HW_ENTER();

    if(func_num > SDIO_FUNC_MAX)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* ����abort��ֵ������func num,Ϊ�˲��ƻ����� */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_IO_ABORT,0,&abort))
    {
        abort |= func_num;
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_SET_ABORT_FAIL;
    }

    /* ����д��ȥ */
    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_IO_ABORT,abort,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_SET_ABORT_FAIL;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_reset
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		sdio reset
******************************************************************************/
uint8_t hw_sdio_reset(void)
{
    uint8_t abort;
    HW_ENTER();

    /* reset����abort�ļĴ�����bit3 */
    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_IO_ABORT,0,&abort))
    {
        abort |= 0x8;
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_RESET_FAIL;
    }

    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_IO_ABORT,abort,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_RESET_FAIL;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_set_bus_width
 * ����:  		bus_width(IN)			-->���ݿ��
 * ����ֵ: 	����ִ�н��
 * ����:		����SDIO�����ݿ��
 				00b: 1-bit
				01b: Reserved
				10b: 4-bit bus
				11b: 8-bit bus (only for embedded SDIO)
******************************************************************************/
uint8_t hw_sdio_set_bus_width(uint8_t bus_width)
{
    uint8_t width;
    HW_ENTER();

    if(!hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_BUS_CONTROL,0,&width))
    {
        /* ����������趨���ݿ�� */
        width &= ~0x3;
        width |= bus_width;
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_SET_BUS_WIDTH_FAIL;
    }

    if(hw_sdio_cmd52(SDIO_EXCU_WRITE,SDIO_FUNC_0,SDIO_CCCR_BUS_CONTROL,width,NULL))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_SET_BUS_WIDTH_FAIL;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_get_bus_width
 * ����:  		bus_width(OUT)			-->���ݿ��
 * ����ֵ: 	����ִ�н��
 * ����:		��ȡDIO�����ݿ��
 				00b: 1-bit
				01b: Reserved
				10b: 4-bit bus
				11b: 8-bit bus (only for embedded SDIO)
******************************************************************************/
uint8_t hw_sdio_get_bus_width(uint8_t *bus_width)
{
    HW_ENTER();
    if(!bus_width)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    if(hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0,SDIO_CCCR_BUS_CONTROL,0,bus_width))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_GET_BUS_WIDTH_FAIL;
    }
    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_get_cis_ptr
 * ����:  		func_num(IN)				-->func���
 				bus_width(OUT)			-->���ݿ��
 * ����ֵ: 	����ִ�н��
 * ����:		����func��Ż�ȡCISָ��
******************************************************************************/
uint8_t hw_sdio_get_cis_ptr(uint8_t func_num,uint32_t *ptr_address)
{
    uint8_t index;
    HW_ENTER();

    uint32_t prt_temp = 0;
    if(func_num > SDIO_FUNC_MAX || (!ptr_address))
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    /* CIS��ָ��ֱ�Ϊ:0x9,0xa,0xb����ȡ�������������CISָ�� */
    for (index = 0; index < 3; index++)
    {
        uint8_t x;

        hw_sdio_cmd52(SDIO_EXCU_READ, SDIO_FUNC_0,SDIO_FBR_BASE(func_num) + SDIO_CCCR_CIS_PTR + index, 0, &x);

        prt_temp |= x << (index * 8);
    }

    *ptr_address = prt_temp;
    HW_LEAVE();
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_set_blk_size
 * ����:  		func_num(IN)				-->func���
 				blk_size(IN)				-->����block size
 * ����ֵ: 	����ִ�н��
 * ����:		�����ض���func��block size
******************************************************************************/
uint8_t hw_sdio_set_blk_size(uint8_t func_num,uint16_t blk_size)
{
    HW_ENTER();

    /* ����block size */
    hw_sdio_cmd52(SDIO_EXCU_WRITE, SDIO_FUNC_0,SDIO_FBR_BASE(func_num) + SDIO_CCCR_BLK_SIZE, blk_size & 0xff, NULL);
    hw_sdio_cmd52(SDIO_EXCU_WRITE, SDIO_FUNC_0,SDIO_FBR_BASE(func_num) + SDIO_CCCR_BLK_SIZE+1, (blk_size >> 8)&0xff, NULL);

    /* ���µ��ض���func num �ṹ���� */
    if((phw_sdio_core->func)[func_num])
    {
        (phw_sdio_core->func)[func_num]->cur_blk_size = blk_size;
    }
    else
    {
        return HW_ERR_SDIO_INVALID_FUNC_NUM;
    }
    HW_LEAVE();
    return  HW_ERR_OK;

}

/******************************************************************************
 *	������:	hw_sdio_get_blk_size
 * ����:  		func_num(IN)				-->func���
 				blk_size(OUT)				-->block size
 * ����ֵ: 	����ִ�н��
 * ����:		��ȡblock size
 				NOTED:����ֱ�Ӷ�ȡ�Ĵ���������ͨ��func�Ľṹ����
******************************************************************************/
uint8_t hw_sdio_get_blk_size(uint8_t func_num,uint16_t *blk_size)
{
    HW_ENTER();

    if(!blk_size)
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_PARA;
    }

    if((phw_sdio_core->func)[func_num])
    {
        *blk_size = (phw_sdio_core->func)[func_num]->cur_blk_size;
    }
    else
    {
        HW_LEAVE();
        return HW_ERR_SDIO_INVALID_FUNC_NUM;
    }

    HW_LEAVE();
    return  HW_ERR_OK;
}


/******************************************************************************
 *	������:	hw_sdio_cmd52
 * ����:  		write(IN)			-->ִ�в�����read or write
 				func_num(IN)		-->func�ı��
 				address(IN)		-->address��ַ
 				para(IN)			-->Ҫд�Ĳ���
 				resp(OUT)			-->��Ҫ���ص�����
 * ����ֵ: 	����ִ�н��
 * ����:		ִ��CMD52�Ķ���
******************************************************************************/
uint8_t hw_sdio_cmd52(uint8_t write,uint8_t func_num,uint32_t address,uint8_t para,uint8_t *resp)
{
    uint8_t error_status;
    uint8_t response;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;

    SDIO_CmdInitStructure.SDIO_Argument = write ? 0x80000000 : 0x00000000;
    SDIO_CmdInitStructure.SDIO_Argument |= func_num << 28;
    SDIO_CmdInitStructure.SDIO_Argument |= (write && resp) ? 0x08000000 : 0x00000000;
    SDIO_CmdInitStructure.SDIO_Argument |= address << 9;
    SDIO_CmdInitStructure.SDIO_Argument |= para;

    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CMD52;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;

    SDIO_SendCommand(&SDIO_CmdInitStructure);
    /* �ȴ�������� */
    while (SDIO_GetFlagStatus(SDIO_FLAG_CMDACT) == SET);
    error_status = hw_sdio_check_err();

    if (HW_ERR_OK != error_status)
    {
        return  HW_ERR_SDIO_CMD52_FAIL;
    }

    response = SDIO_GetResponse(SDIO_RESP1) & 0xff;
    if((!write) && resp)
    {
        *resp = response;
    }
    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_cmd52
 * ����:  		write(IN)			-->ִ�в�����read or write
 				func_num(IN)		-->func�ı��
 				address(IN)		-->address��ַ
 				incr_addr(IN)		-->��ַ�Ƿ��ۼ�
 				buf(IN/OUT)		-->���������д����ô�˲�������Ҫwrite��buffer
 										��������Ƕ�����ô�˲�������read���ص�buffer
 				size(IN)			-->������д��size
 * ����ֵ: 	����ִ�н��
 * ����:		ִ��CMD53�Ķ���
******************************************************************************/
uint8_t hw_sdio_cmd53(uint8_t write, uint8_t func_num,uint32_t address, uint8_t incr_addr, uint8_t *buf,uint32_t size)
{
    uint16_t func_cur_blk_size;

    if((phw_sdio_core->func)[func_num])
    {
        func_cur_blk_size = (phw_sdio_core->func)[func_num]->cur_blk_size;
        if(func_cur_blk_size == 0)
        {
            return HW_ERR_SDIO_BLK_SIZE_ZERO;
        }
    }
    else
    {
        return HW_ERR_SDIO_INVALID_FUNC_NUM;
    }

    if(write)
    {
        /* CMD53 write */
        hw_sdio_cmd53_write(func_num,address,incr_addr,buf,size,func_cur_blk_size);
    }
    else
    {
        /* CMD53 read */
        hw_sdio_cmd53_read(func_num,address,incr_addr,buf,size,func_cur_blk_size);
    }

    return  HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_core_init
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		sdio core�ĳ�ʼ��
******************************************************************************/
static uint8_t hw_sdio_core_init(void)
{
    hw_memset(&hw_sdio_func,0,sizeof(sdio_func_t)*SDIO_FUNC_MAX);
    hw_memset(&hw_sdio_core,0,sizeof(sdio_core_t));
    hw_sdio_func[SDIO_FUNC_0].func_status = FUNC_INT_ENABLE;
    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_parse_r6
 * ����:  		r6(IN)			-->R6�����
 				rca(OUT)		-->rca�ķ���ֵ
 * ����ֵ: 	����ִ�н��
 * ����:		����R6��response
******************************************************************************/
static uint8_t hw_sdio_parse_r6(uint32_t r6,uint32_t *rca)
{
    HW_ENTER();
    if(rca)
    {
        *rca = RCA_IN_R6(r6);
        HW_LEAVE();
        return HW_ERR_OK;
    }
    HW_LEAVE();
    return HW_ERR_SDIO_INVALID_PARA;
}

/******************************************************************************
 *	������:	hw_sdio_parse_r4
 * ����:  		r4(IN)			-->R4�����
 * ����ֵ: 	����ִ�н��
 * ����:		����R4����Ҫ��עfunc������
******************************************************************************/
static uint8_t hw_sdio_parse_r4(uint32_t r4)
{
    HW_ENTER();
    uint32_t index = 0;

    phw_sdio_core->func_total_num = FUNC_NUM_IN_R4(r4);
    for(index = 0; index < phw_sdio_core->func_total_num; index++)
    {
        (phw_sdio_core->func)[index] = &hw_sdio_func[index];
        hw_sdio_func[index].func_num = index;
    }

    HW_LEAVE();
    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_cis_read_parse
 * ����:  		func_num(IN)			-->func���
 				cis_ptr(IN)			-->CISָ��
 * ����ֵ: 	����ִ�н��
 * ����:		��ȡCIS���ݲ�������������Ҫ�����ݴ洢��core�Ľṹ����
******************************************************************************/
static uint8_t hw_sdio_cis_read_parse(uint8_t func_num,uint32_t cis_ptr)
{
    /* SDIOЭ����������һ�仰 :No SDIO card tuple can be longer than 257 bytes
     * 1 byte TPL_CODE + 1 byte TPL_LINK +
     *	FFh byte tuple body (and this 257 bytetuple ends the chain)
     * �������Ƕ������������255
     */
    uint8_t data[255];
    uint8_t index,len;
    uint8_t tpl_code = CISTPL_NULL;
    uint8_t tpl_link;
    uint32_t cis_ptr_temp = cis_ptr;

    HW_ENTER();
    while (tpl_code != CISTPL_END)
    {
        hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0, cis_ptr_temp++,0,&tpl_code);
        if (tpl_code == CISTPL_NULL)
            continue;

        /* ��������ݵĴ�С */
        hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0, cis_ptr_temp++,0,&tpl_link);

        for (index = 0; index < tpl_link; index++)
            hw_sdio_cmd52(SDIO_EXCU_READ,SDIO_FUNC_0, cis_ptr_temp + index,0,&data[index]);

        switch (tpl_code)
        {
        case CISTPL_VERS_1:
            HW_DEBUG("Product Information:");
            for (index = 2; data[index] != 0xff; index += len + 1)
            {
                // ���������ַ���
                len = hw_strlen((char *)data + index);
                if (len != 0)
                    HW_DEBUG(" %s", data + index);
            }
            HW_DEBUG("\n");
            break;
        case CISTPL_MANFID:
            // 16.6 CISTPL_MANFID: Manufacturer Identification String Tuple
            HW_DEBUG("Manufacturer Code: 0x%04x\n", *(uint16_t *)data); // TPLMID_MANF
            HW_DEBUG("Manufacturer Information: 0x%04x\n", *(uint16_t *)(data + 2)); // TPLMID_CARD
            phw_sdio_core->manf_code = *(uint16_t *)data;
            phw_sdio_core->manf_info = *(uint16_t *)(data + 2);
            break;
        case CISTPL_FUNCID:
            // 16.7.1 CISTPL_FUNCID: Function Identification Tuple
            HW_DEBUG("Card Function Code: 0x%02x\n", data[0]); // TPLFID_FUNCTION
            HW_DEBUG("System Initialization Bit Mask: 0x%02x\n", data[1]); // TPLFID_SYSINIT
            break;
        case CISTPL_FUNCE:
            // 16.7.2 CISTPL_FUNCE: Function Extension Tuple
            if (data[0] == 0)
            {
                // 16.7.3 CISTPL_FUNCE Tuple for Function 0 (Extended Data 00h)
                HW_DEBUG("Maximum Block Size case1: func: %d,size %d\n", func_num, *(uint16_t *)(data + 1));
                HW_DEBUG("Maximum Transfer Rate Code: 0x%02x\n", data[3]);
                if((phw_sdio_core->func)[func_num])
                {
                    (phw_sdio_core->func)[func_num]->max_blk_size = *(uint16_t *)(data + 1);
                }
            }
            else
            {
                // 16.7.4 CISTPL_FUNCE Tuple for Function 1-7 (Extended Data 01h)
                HW_DEBUG("Maximum Block Size case2 func: %d,size %d\n", func_num,*(uint16_t *)(data + 0x0c)); // TPLFE_MAX_BLK_SIZE
                if((phw_sdio_core->func)[func_num])
                {
                    (phw_sdio_core->func)[func_num]->max_blk_size = *(uint16_t *)(data + 0x0c);
                }

            }
            break;
        default:
            HW_DEBUG("[CIS Tuple 0x%02x] addr=0x%08x size=%d\n", tpl_code, cis_ptr_temp - 2, tpl_link);
#if HW_DEBUG_ENABLE > 0
            hw_hex_dump(data, tpl_link);
#endif
        }

        if (tpl_link == 0xff)
            break; // ��TPL_LINKΪ0xffʱ˵����ǰ���Ϊβ�ڵ�
        cis_ptr_temp += tpl_link;
    }

    HW_LEAVE();
    return HW_ERR_OK;
}


static uint8_t hw_sdio_set_dblocksize(uint32_t *struct_dblocksize,uint32_t block_size)
{
    uint32_t dblock_size;
    switch (block_size)
    {
    case 1:
        dblock_size = SDIO_DataBlockSize_1b;
        break;
    case 2:
        dblock_size = SDIO_DataBlockSize_2b;
        break;
    case 4:
        dblock_size = SDIO_DataBlockSize_4b;
        break;
    case 8:
        dblock_size = SDIO_DataBlockSize_8b;
        break;
    case 16:
        dblock_size = SDIO_DataBlockSize_16b;
        break;
    case 32:
        dblock_size = SDIO_DataBlockSize_32b;
        break;
    case 64:
        dblock_size = SDIO_DataBlockSize_64b;
        break;
    case 128:
        dblock_size = SDIO_DataBlockSize_128b;
        break;
    case 256:
        dblock_size = SDIO_DataBlockSize_256b;
        break;
    case 512:
        dblock_size = SDIO_DataBlockSize_512b;
        break;
    case 1024:
        dblock_size = SDIO_DataBlockSize_1024b;
        break;
    case 2048:
        dblock_size = SDIO_DataBlockSize_2048b;
        break;
    case 4096:
        dblock_size = SDIO_DataBlockSize_4096b;
        break;
    case 8192:
        dblock_size = SDIO_DataBlockSize_8192b;
        break;
    case 16384:
        dblock_size = SDIO_DataBlockSize_16384b;
        break;
    }
    *struct_dblocksize = dblock_size;
    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_check_err
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		��֤��ʲô���󣬲��������Ӧ��flag
******************************************************************************/
static uint8_t hw_sdio_check_err()
{
    uint8_t err = HW_ERR_OK;

    if (SDIO_GetFlagStatus(SDIO_FLAG_CCRCFAIL) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        err++;
        HW_DEBUG("L:%d, %s: CMD%d CRC failed!\n", __LINE__, __func__, SDIO->CMD & SDIO_CMD_CMDINDEX);
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_CTIMEOUT) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        err++;
        HW_DEBUG("L:%d, %s: CMD%d timeout!\n", __LINE__, __func__, SDIO->CMD & SDIO_CMD_CMDINDEX);
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        err++;
        HW_DEBUG("L:%d, %s: data CRC failed!\n", __LINE__, __func__);
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        err++;
        HW_DEBUG("L:%d, %s: data timeout!\n", __LINE__, __func__);
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        err++;
        HW_DEBUG("L:%d, %s: start bit error!\n", __LINE__, __func__);
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
        err++;
        HW_DEBUG("L:%d, %s: data underrun!\n", __LINE__, __func__);
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) == SET)
    {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        err++;
        HW_DEBUG("L:%d, %s: data overrun!\n", __LINE__, __func__);
    }

    return err;
}

/******************************************************************************
 *	������:	hw_sdio_cmd3
 * ����:  		para(IN)		-->����cmd3�Ĳ���
 				resp			-->cmd3�ķ���ֵ
 * ����ֵ: 	����ִ�н��
 * ����:		����cmd3
******************************************************************************/
static uint8_t hw_sdio_cmd3(uint32_t para,uint32_t *resp)
{
    uint8_t error_status;
    uint32_t response;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;

    SDIO_CmdInitStructure.SDIO_Argument = para;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CMD3;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    /* �ȴ�������� */
    while (SDIO_GetFlagStatus(SDIO_FLAG_CMDACT) == SET);
    error_status = hw_sdio_check_err();

    if (HW_ERR_OK != error_status)
    {
        return error_status;
    }

    /* ��ȡ��response�Ľ�� */
    response = SDIO_GetResponse(SDIO_RESP1);
    if (resp)
    {
        *resp = response;
    }

    return (error_status);
}

/******************************************************************************
 *	������:	hw_sdio_cmd5
 * ����:  		para(IN)			-->���
 				resp(OUT)			-->����ֵ
 				retry_max(IN)		-->����Դ���
 * ����ֵ: 	����ִ�н��
 * ����:		����cmd5
******************************************************************************/
static uint8_t hw_sdio_cmd5(uint32_t para,uint32_t *resp,uint32_t retry_max)
{
    uint32_t index;
    uint32_t response = 0;
    uint8_t error_status;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    
    retry_max = 6;

    HW_ENTER();
    SDIO_CmdInitStructure.SDIO_Argument = para;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CMD5 ;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    for (index = 0; index < retry_max; index++)
    {
        SDIO_SendCommand(&SDIO_CmdInitStructure);
        /* �ȴ�������� */
        while (SDIO_GetFlagStatus(SDIO_FLAG_CMDACT) == SET);
        error_status = hw_sdio_check_err();

        if ((HW_ERR_OK != error_status) && (1 != error_status)) // ignore cmd5 cmd crc error
        {
            continue;
        }
        
        if (error_status == 1)
        {
            error_status = 0; // ignore cmd5 cmd crc error for ln
            HW_DEBUG("[Note] Ignore CMD5 CRC error!\r\n");
        }

        response = SDIO_GetResponse(SDIO_RESP1);

        /* �ж��Ƿ�OK */
//        if(C_IN_R4(response)) // ignore cmd5 respones`s 'C' bit.
        {
            if (resp)
            {
                *resp = response;
            }
            break;
        }
    }

    HW_LEAVE();
    return error_status;
}


/******************************************************************************
 *	������:	hw_sdio_cmd7
 * ����:  		para(IN)			-->���
 				resp(OUT)			-->����ֵ
 * ����ֵ: 	����ִ�н��
 * ����:		����cmd7
******************************************************************************/
static uint8_t hw_sdio_cmd7(uint32_t para,uint32_t *resp)
{
    uint8_t error_status;
    uint32_t response;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;

    HW_ENTER();
    /* Send CMD7 SDIO_SEL_DESEL_CARD */
    SDIO_CmdInitStructure.SDIO_Argument = para;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CMD7;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    /* �ȴ�������� */
    while (SDIO_GetFlagStatus(SDIO_FLAG_CMDACT) == SET);
    error_status = hw_sdio_check_err();

    if (HW_ERR_OK != error_status)
    {
        return error_status;
    }
    /* ��ȡ���ؽ�� */
    response = SDIO_GetResponse(SDIO_RESP1);
    if (resp)
    {
        *resp = response;
    }

    HW_ENTER();
    return (error_status);
}


/******************************************************************************
 *	������:	hw_sdio_cmd53_read
 * ����:  		func_num(IN)			-->func���
 				address(IN)			-->Ҫ��ȡ�ĵ�ַ
 				incr_addr(IN)			-->��ַ�Ƿ��ۼ�
 				buf(OUT)				-->���ݷ���buffer
 				size(IN)				-->Ҫ��ȡ��size
 				cur_blk_size(IN)		-->��ǰ��func��ŵ�block size
 * ����ֵ: 	����ִ�н��
 * ����:		ִ��CMD53��read������block mode
******************************************************************************/
static uint8_t hw_sdio_cmd53_read(uint8_t func_num,uint32_t address, uint8_t incr_addr, uint8_t *buf,uint32_t size,uint16_t cur_blk_size)
{
    uint8_t error_status;
    uint32_t remain_size = size;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    hw_memset(&SDIO_DataInitStructure,0,sizeof(SDIO_DataInitTypeDef));
    hw_memset(&SDIO_CmdInitStructure,0,sizeof(SDIO_CmdInitStructure));
    hw_memset(&DMA_InitStructure,0,sizeof(DMA_InitStructure));

    /* 2.����DMA */
    DMA_DeInit(DMA2_Channel4);
    DMA_Cmd(DMA2_Channel4, DISABLE);
    DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

    /* DMA2 Channel4 Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDIO->FIFO;;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    if(remain_size%cur_blk_size)
    {
        DMA_InitStructure.DMA_BufferSize = (remain_size/cur_blk_size+1)*cur_blk_size/4;
    }
    else
    {
        DMA_InitStructure.DMA_BufferSize = (remain_size/cur_blk_size)*cur_blk_size/4;
    }
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel4, &DMA_InitStructure);
    /* DMA2 Channel4 enable */
    DMA_Cmd(DMA2_Channel4, ENABLE);
    /* 3.����SDIO data�ṹ�� */
    SDIO_DataInitStructure.SDIO_DataTimeOut = SDIO_24M_DATATIMEOUT;
    if(remain_size%cur_blk_size)
    {
        SDIO_DataInitStructure.SDIO_DataLength = (remain_size/cur_blk_size+1)*cur_blk_size;
    }
    else
    {
        SDIO_DataInitStructure.SDIO_DataLength = (remain_size/cur_blk_size)*cur_blk_size;
    }

    hw_sdio_set_dblocksize(&SDIO_DataInitStructure.SDIO_DataBlockSize,cur_blk_size);
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;

    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    /* 1.����CMD53 */
    /* CMD53�����������ʽΪ */
    /* |--RW FLAG--|--FUNC NUM--|--BLK MODE--|--OP MODE--|--REG ADDR--|--BYTE/BLK CNT--| */
    /* |--1  BYTE--|--3   BYTE--|--1   BYTE--|--1  BYTE--|--17  BYTE--|--9      BYTE --| */
    SDIO_CmdInitStructure.SDIO_Argument |= 0x0;					/* CMD53��R/W read��flag */
    SDIO_CmdInitStructure.SDIO_Argument |= func_num << 28;	/* FUNC */
    SDIO_CmdInitStructure.SDIO_Argument |= 0x08000000;			/* Block mode */
    SDIO_CmdInitStructure.SDIO_Argument |= incr_addr ? 0x04000000 : 0x0;	/* OP MODE :1.���� 0,�̶���ַ */
    if(incr_addr)
        SDIO_CmdInitStructure.SDIO_Argument |= (address) << 9;/* REG ADDR,Ҫд��ĵ�ַ */
    else
        SDIO_CmdInitStructure.SDIO_Argument |= address << 9;		/* REG ADDR,Ҫд��ĵ�ַ */

    if(remain_size%cur_blk_size)
    {
        SDIO_CmdInitStructure.SDIO_Argument |= (remain_size/cur_blk_size+1);
    }
    else
    {
        SDIO_CmdInitStructure.SDIO_Argument |= remain_size/cur_blk_size;
    }
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CMD53;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    /* �ȴ�������ɣ������ж��Ƿ��д������ */
    while (SDIO_GetFlagStatus(SDIO_FLAG_CMDREND) == RESET);
    error_status = hw_sdio_check_err();

    if (HW_ERR_OK != error_status)
    {
        return  HW_ERR_SDIO_CMD53_FAIL;
    }
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);

    while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET); 	/* �ȴ�DMA���ͳɹ� */
    SDIO_ClearFlag(SDIO_FLAG_DATAEND);						/* ���������ɱ�־ */
    DMA_ClearFlag(DMA2_FLAG_TC4);								/* ���DMA2 channel4������ɵı�־ */
    DMA_DeInit(DMA2_Channel4);									/* ����ʼ��DMA2 channel4 */
    DMA_Cmd(DMA2_Channel4, DISABLE);							/* �ر�DMA2 channel4 */

    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_sdio_cmd53_write
 * ����:  		func_num(IN)			-->func���
 				address(IN)			-->Ҫ��ȡ�ĵ�ַ
 				incr_addr(IN)			-->��ַ�Ƿ��ۼ�
 				buf(IN)				-->���ݷ���buffer
 				size(IN)				-->Ҫ��ȡ��size
 				cur_blk_size(IN)		-->��ǰ��func��ŵ�block size
 * ����ֵ: 	����ִ�н��
 * ����:		ִ��CMD53��write������block modeд��ȥ
******************************************************************************/
static uint8_t hw_sdio_cmd53_write(uint8_t func_num,uint32_t address, uint8_t incr_addr, uint8_t *buf,uint32_t size,uint16_t cur_blk_size)
{
    uint8_t error_status = 0;
    uint32_t remain_size = size;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    hw_memset(&SDIO_DataInitStructure,0,sizeof(SDIO_DataInitTypeDef));
    hw_memset(&SDIO_CmdInitStructure,0,sizeof(SDIO_CmdInitStructure));
    hw_memset(&DMA_InitStructure,0,sizeof(DMA_InitStructure));

    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    /* 1.����CMD53 */
    /* CMD53�����������ʽΪ */
    /* |--RW FLAG--|--FUNC NUM--|--BLK MODE--|--OP MODE--|--REG ADDR--|--BYTE/BLK CNT--| */
    /* |--1  BYTE--|--3   BYTE--|--1   BYTE--|--1  BYTE--|--17  BYTE--|--9      BYTE --| */
    SDIO_CmdInitStructure.SDIO_Argument = 0x80000000;			/* CMD53��R/W write��flag */
    SDIO_CmdInitStructure.SDIO_Argument |= func_num << 28;	/* FUNC */
    SDIO_CmdInitStructure.SDIO_Argument |= 0x08000000;			/* Block mode */
    SDIO_CmdInitStructure.SDIO_Argument |= incr_addr ? 0x04000000 : 0x0;	/* OP MODE :1.���� 0,�̶���ַ */

    SDIO_CmdInitStructure.SDIO_Argument |= address << 9;		/* REG ADDR,Ҫд��ĵ�ַ */

    if(remain_size%cur_blk_size)
    {
        SDIO_CmdInitStructure.SDIO_Argument |= (remain_size/cur_blk_size+1);
    }
    else
    {
        SDIO_CmdInitStructure.SDIO_Argument |= remain_size/cur_blk_size;
    }

    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_CMD53;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;

    SDIO_SendCommand(&SDIO_CmdInitStructure);
    while (SDIO_GetFlagStatus(SDIO_FLAG_CMDREND) == RESET);
    error_status = hw_sdio_check_err();

    if (HW_ERR_OK != error_status)
    {
        return  HW_ERR_SDIO_CMD53_FAIL;
    }
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    /* 2.����DMA */
    /* DMA2 Channel4 enable */

    DMA_DeInit(DMA2_Channel4);
    DMA_Cmd(DMA2_Channel4, DISABLE);
    DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

    /* DMA2 Channel4 Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDIO->FIFO;;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;

    if(remain_size%cur_blk_size)
    {
        DMA_InitStructure.DMA_BufferSize = (remain_size/cur_blk_size+1)*cur_blk_size/4;
    }
    else
    {
        DMA_InitStructure.DMA_BufferSize = (remain_size/cur_blk_size)*cur_blk_size/4;
    }

    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA2_Channel4, &DMA_InitStructure);
    DMA_Cmd(DMA2_Channel4, ENABLE);

    /* 3.����SDIO data�ṹ�� */
    SDIO_DataInitStructure.SDIO_DataTimeOut = SDIO_24M_DATATIMEOUT;

    if(remain_size%cur_blk_size)
    {
        SDIO_DataInitStructure.SDIO_DataLength = (remain_size/cur_blk_size+1)*cur_blk_size;
    }
    else
    {
        SDIO_DataInitStructure.SDIO_DataLength = (remain_size/cur_blk_size)*cur_blk_size;
    }


    hw_sdio_set_dblocksize(&SDIO_DataInitStructure.SDIO_DataBlockSize,cur_blk_size);
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;

    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;

    SDIO_DataConfig(&SDIO_DataInitStructure);


    while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET); /* �ȴ�DMA���ͳɹ� */
    SDIO_ClearFlag(SDIO_FLAG_DATAEND);						/* ���������ɱ�־ */
    SDIO_ClearFlag(SDIO_FLAG_DBCKEND);						/* �������/�������ݿ� */
    DMA_ClearFlag(DMA2_FLAG_TC4);								/* ���DMA2 channel4������ɵı�־ */
    DMA_DeInit(DMA2_Channel4);									/* ����ʼ��DMA2 channel4 */
    DMA_Cmd(DMA2_Channel4, DISABLE);							/* �ر�DMA2 channel4 */



    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	hw_chip_reset
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		chip reset,�˲�����WIFIģ���PDN���� PD5
******************************************************************************/
uint8_t hw_chip_reset()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    CHIP_RESET_LOW;
    hw_delay_ms(10);
    CHIP_RESET_HIGH;
    hw_delay_ms(10);
    return HW_ERR_OK;
}

/******************************************************************************
 *	������:	SDIO_IRQHandler
 * ����:  		NULL
 * ����ֵ: 	NULL
 * ����:		SDIO�жϴ�����
******************************************************************************/
void SDIO_IRQHandler(void)
{
    if(SDIO_GetITStatus(SDIO_IT_CCRCFAIL) == SET)
    {
        /* ��������CRC���� */
        SDIO_ClearITPendingBit(SDIO_IT_CCRCFAIL);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_CCRCFAIL OCCUR\n");
    }
    if(SDIO_GetITStatus(SDIO_IT_DCRCFAIL) == SET)
    {
        /* ��������CRC���� */
        SDIO_ClearITPendingBit(SDIO_IT_DCRCFAIL);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_DCRCFAIL OCCUR\n");
    }
    if(SDIO_GetITStatus(SDIO_IT_CTIMEOUT) == SET)
    {
        /* �������ʱ���� */
        SDIO_ClearITPendingBit(SDIO_IT_CTIMEOUT);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_CTIMEOUT OCCUR\n");
    }
    if(SDIO_GetITStatus(SDIO_IT_DTIMEOUT) == SET)
    {
        /* �������ݳ�ʱ���� */
        SDIO_ClearITPendingBit(SDIO_IT_DTIMEOUT);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_DTIMEOUT OCCUR\n");
    }
    if(SDIO_GetITStatus( SDIO_IT_TXUNDERR) == SET)
    {
        /* ��������FIFO������� */
        SDIO_ClearITPendingBit(SDIO_IT_TXUNDERR);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_TXUNDERR OCCUR\n");
    }
    if(SDIO_GetITStatus(SDIO_IT_RXOVERR) == SET)
    {
        /* ��������FIFO������� */
        SDIO_ClearITPendingBit(SDIO_IT_RXOVERR);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_RXOVERR OCCUR\n");
    }
    if(SDIO_GetITStatus(SDIO_IT_STBITERR) == SET)
    {
        /* �ڿ�����ģʽ��û�������������ź��ϼ�⵽��ʼλ */
        SDIO_ClearITPendingBit(SDIO_IT_STBITERR);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_STBITERR OCCUR\n");
    }
    if(SDIO_GetITStatus(SDIO_IT_SDIOIT) == SET)
    {
        /* �յ�SDIO�ж� */
        SDIO_ClearITPendingBit(SDIO_IT_SDIOIT);
        HW_DEBUG("SDIO_IRQHandler:SDIO_IT_SDIOIT OCCUR\n");
        //wifi_process_packet();
    }
}

