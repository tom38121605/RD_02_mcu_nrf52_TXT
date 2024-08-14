/*! ----------------------------------------------------------------------------
 * @file     cmd_fn.c
 * @brief    collection of executables functions
 *
 * @author   Decawave
 *
 * @attention Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "port_config.h"
#include "instance.h"
#include "port_stdio.h"
#include "production_uwb.h"
#include "translate.h"
#include "version.h"
#include "cmd.h"
#include "deca_version.h"

#include "cmd_fn.h"
//#include "main.h"

#define MAX_STR_SIZE        512
#define DEVICE_NAME_STR                         ("\"DW3100 Production FW\"")

//-----------------------------------------------------------------------------
static const char CMD_FN_RET_OK[] = "ok\r\n";
static const char CMD_FN_RET_ERR[] = "er\r\n";

static const command_t known_commands[];

const command_t* cmd_fn_get_list(void)
{
    return known_commands;
}



/*******************************************************************************
 *
 *                          f_xx "command" FUNCTIONS
 *
 * REG_FN(f_tag) macro will create a function
 *
 * const char *f_tag(char *text, param_block_t *pbss, int val)
 *
 * */


//-----------------------------------------------------------------------------

REG_COMMON_FN(f_prod_ucw, cmd_uwb_cw_mode)
REG_COMMON_FN(f_prod_urst, cmd_uwb_reset)
REG_COMMON_FN(f_prod_uxt, cmd_uwb_set_xtaltrim)
REG_COMMON_FN(f_prod_uht, cmd_uwb_hw_test)
REG_COMMON_FN(f_prod_ups, cmd_uwb_param_set)
REG_COMMON_FN(f_prod_upps, cmd_uwb_pdoa_param_set)
REG_COMMON_FN(f_prod_ucf, cmd_uwb_cf_mode)
REG_COMMON_FN(f_prod_utpg, cmd_uwb_get_tx_power)
REG_COMMON_FN(f_prod_utp, cmd_uwb_set_tx_power)
REG_COMMON_FN(f_prod_uts, cmd_uwb_uts)
REG_COMMON_FN(f_prod_uee, cmd_uwb_eventcounter_enable)
REG_COMMON_FN(f_prod_ued, cmd_uwb_eventcounter_disable)
REG_COMMON_FN(f_prod_uep, cmd_uwb_eventcounter_print)
REG_COMMON_FN(f_prod_urx, cmd_uwb_rx)
REG_COMMON_FN(f_prod_urs, cmd_uwb_reg_set)
REG_COMMON_FN(f_prod_urg, cmd_uwb_reg_get)
REG_COMMON_FN(f_prod_utx, cmd_uwb_tx)
REG_COMMON_FN(f_prod_usa, cmd_uwb_set_ant_cfg)
REG_COMMON_FN(f_prod_uga, cmd_uwb_get_ant_cfg)
REG_COMMON_FN(f_prod_uof, cmd_uwb_off)
REG_COMMON_FN(f_prod_utvb, cmd_read_temp_vbat)
REG_COMMON_FN(f_prod_utwri, cmd_twr_initiator)
REG_COMMON_FN(f_prod_utwrr, cmd_twr_responder)
REG_COMMON_FN(f_prod_uod, cmd_dump_otp)
REG_COMMON_FN(f_prod_uow, cmd_otp_write)
REG_COMMON_FN(f_prod_uotp, cmd_uwb_uotp)
REG_COMMON_FN(f_prod_upg, cmd_uwb_upg)
REG_COMMON_FN(f_prod_udw, cmd_uwb_udw)
REG_COMMON_FN(f_prod_urd, cmd_uwb_urd)
REG_COMMON_FN(f_prod_sleep, cmd_uwb_sleep)
REG_COMMON_FN(f_prod_idle_pll, cmd_uwb_idle_pll)
REG_COMMON_FN(f_prod_idle_rc, cmd_uwb_idle_rc)
REG_COMMON_FN(f_prod_sync_pin, cmd_uwb_sync_test)

//-----------------------------------------------------------------------------
// Parameters change section
void cfg_update(param_block_t * pbss)
{
    instance_config(pbss);
    port_config_save();
}

REG_FN(f_chan)
{
    int tmp = val;
    const char * ret = NULL;
    int ch_old = pbss->dwt_config.chan;
    
    if(cmd_get_argc() != 2)
    {
        return "usage: CHAN <5, 9>\r\n";
    }

    if(tmp>=0)
    {
      pbss->dwt_config.chan = tmp;
      ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);
    
    if(ch_old != tmp)
    {
        cmd_uwb_reset();
    }

    return (ret);
}

REG_FN(f_plen)
{
    int tmp = plen_to_deca(val);
    const char * ret = NULL;
    
    if(cmd_get_argc() != 2)
    {
        return "usage: PLEN <32 ~ 4096>\r\n";
    }

    if(tmp>=0)
    {
      pbss->dwt_config.txPreambLength = (uint16_t)(tmp);
      stdto_update();
      ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);

    return (ret);
}

REG_FN(f_rxPAC)
{
    int tmp = pac_to_deca(val);
    const char * ret = NULL;
    
    if(cmd_get_argc() != 2)
    {
        return "usage: PAC <4,8,16,32>\r\n";
    }

    if(tmp>=0)
    {
      pbss->dwt_config.rxPAC = (uint8_t)(tmp);
      stdto_update();
      ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);


    return (ret);
}

REG_FN(f_code)
{    
    if(cmd_get_argc() != 2)
    {
        return "usage: CODE <9,10,11,12>\r\n";
    }

    pbss->dwt_config.txCode = (uint8_t)(val);
    pbss->dwt_config.rxCode = (uint8_t)(val);

    cfg_update(pbss);

    return (CMD_FN_RET_OK);
}

REG_FN(f_sfd)
{
    const char * ret = NULL;
    
    if(cmd_get_argc() != 2)
    {
        return "usage: SFD <0,1,2,3>\r\n";
    }

    if((val >= 0) && (val <= 3))
    {
        pbss->dwt_config.sfdType = (uint8_t)(val);
        stdto_update();
        ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);

    return (ret);
}    
 
REG_FN(f_bitRate)
{    
    if(cmd_get_argc() != 2)
    {
        return "usage: BITR <850,6810>\r\n";
    }

    int tmp = bitrate_to_deca(val);
    const char * ret = NULL;

    if(tmp>=0)
    {
        pbss->dwt_config.dataRate = (uint8_t)(tmp);
        ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);

    return (ret);
}     

REG_FN(f_phrMode)
{    
    if(cmd_get_argc() != 2)
    {
        return "usage: PHRM <0,1>\r\n";
    }

    pbss->dwt_config.phrMode = (val == 0)?(0):(1);

    cfg_update(pbss);

    return (CMD_FN_RET_OK);
}     

REG_FN(f_phrRate)
{    
    if(cmd_get_argc() != 2)
    {
        return "usage: PHRR <0,1>\r\n";
    }
    pbss->dwt_config.phrRate = (val == 0)?(0):(1);

    cfg_update(pbss);

    return (CMD_FN_RET_OK);
}

REG_FN(f_stsMode)
{
    const char * ret = NULL;    
    if(cmd_get_argc() != 2)
    {
        return "usage: STSM <0 ~ 0xFF>\r\n";
    }

    if((val >= 0) && (val <= DWT_STS_CONFIG_MASK))
    {
        pbss->dwt_config.stsMode = val;
        ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);

    return (ret);
}

REG_FN(f_stsLen)
{
    int tmp = slen_to_deca(val);
    const char * ret = NULL;
    if(cmd_get_argc() != 2)
    {
        return "usage: STSL <32 ~ 2048>\r\n";
    }

    if(tmp>=0)
    {
      pbss->dwt_config.stsLength = (uint16_t)(tmp);
      ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);

    return (ret);
}

REG_FN(f_pdoaMode)
{
    const char * ret = NULL;
    if(cmd_get_argc() != 2)
    {
        return "usage: PDOA <0,1,3>\r\n";
    }
    if((val == DWT_PDOA_M0) || (val == DWT_PDOA_M1) || (val == DWT_PDOA_M3))
    {
        pbss->dwt_config.pdoaMode = val;
        ret = CMD_FN_RET_OK;
    }

    cfg_update(pbss);

    return (ret);
}
   
REG_FN(f_tag)
{
    return "work in progress... \r\n";//(CMD_FN_RET_OK);
}      
 
REG_FN(f_node)
{
    return "work in progress... \r\n";//(CMD_FN_RET_OK);
}    




//-----------------------------------------------------------------------------
// Communication /  user statistics section

/* @brief
 * */
REG_FN(f_decaTDoATag)
{
    const char ver[] = FULL_VERSION;

    char str[MAX_STR_SIZE];

    int  hlen;

    hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

    sprintf(&str[strlen(str)],"{\"Info\":{\r\n");
    sprintf(&str[strlen(str)],"\"Device\":%s,\r\n", DEVICE_NAME_STR);
    sprintf(&str[strlen(str)],"\"Version\":\"%s\",\r\n", ver);
    sprintf(&str[strlen(str)],"\"Build\":\"%s %s\",\r\n", __DATE__, __TIME__ );
    sprintf(&str[strlen(str)],"\"Driver\":\"%s\"}}", DW3000_DEVICE_DRIVER_VER_STRING );
    sprintf(&str[2],"%04X",strlen(str)-hlen); //add formatted 4X of length, this will erase first '{'
    str[hlen]='{';                            //restore the start bracket
    port_stdio_transmit((uint8_t*)str, strlen(str));
    port_stdio_transmit((uint8_t*)"\r\n", 2);

    return CMD_FN_RET_OK;
}

//-----------------------------------------------------------------------------

/*
 * @brief   show current UWB parameters in JSON format
 *
 * */
REG_FN(f_jstat)
{

    char str[MAX_STR_SIZE];
    int  hlen;

    hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
    sprintf(&str[strlen(str)],"{\"UWB PARAM\":{\r\n");
    sprintf(&str[strlen(str)],"\"CHAN\":%d,\r\n", pbss->dwt_config.chan);
    sprintf(&str[strlen(str)],"\"PLEN\":%d,\r\n",deca_to_plen(pbss->dwt_config.txPreambLength));
    sprintf(&str[strlen(str)],"\"DATARATE\":%d,\r\n",deca_to_bitrate(pbss->dwt_config.dataRate));
    sprintf(&str[strlen(str)],"\"TXCODE\":%d,\r\n",pbss->dwt_config.txCode);
    sprintf(&str[strlen(str)],"\"PAC\":%d,\r\n", deca_to_pac (pbss->dwt_config.rxPAC));
    sprintf(&str[strlen(str)],"\"SFD\":%d,\r\n",pbss->dwt_config.sfdType);
    sprintf(&str[strlen(str)],"\"PHRMODE\":%d,\r\n",pbss->dwt_config.phrMode);
    sprintf(&str[strlen(str)],"\"phrRate\":%d,\r\n",pbss->dwt_config.phrRate);
    sprintf(&str[strlen(str)],"\"sfdTO\":%d,\r\n",pbss->dwt_config.sfdTO);
    sprintf(&str[strlen(str)],"\"stsMode\":%d,\r\n",pbss->dwt_config.stsMode);
    sprintf(&str[strlen(str)],"\"stsLength\":%d,\r\n",deca_to_slen(pbss->dwt_config.stsLength));
    sprintf(&str[strlen(str)],"\"pdoaMode\":%d,\r\n",pbss->dwt_config.pdoaMode);
    sprintf(&str[strlen(str)],"\"ch5: pgdly\":0x%02x,\r\n",pbss->dw_dev.tx_config[0].PGdly);
    sprintf(&str[strlen(str)],"\"ch5: power\":0x%08lx,\r\n",pbss->dw_dev.tx_config[0].power);
    sprintf(&str[strlen(str)],"\"ch9: pgdly\":0x%02x,\r\n",pbss->dw_dev.tx_config[1].PGdly);
    sprintf(&str[strlen(str)],"\"ch9: power\":0x%08lx,\r\n",pbss->dw_dev.tx_config[1].power);
    sprintf(&str[strlen(str)],"\"DEVID\":0x%08lx,\r\n", pbss->dw_dev.dev_id);
    sprintf(&str[strlen(str)],"\"TAGIDSET\":%d,\r\n",pbss->tagIDset);
    sprintf(&str[strlen(str)],"\"TAGID\":0x%02x%02x%02x%02x%02x%02x%02x%02x}}",
            pbss->tagID[7], pbss->tagID[6], pbss->tagID[5], pbss->tagID[4],
            pbss->tagID[3], pbss->tagID[2], pbss->tagID[1], pbss->tagID[0]);
    sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
    str[hlen]='{';                            //restore the start bracket
    sprintf(&str[strlen(str)],"\r\n");
    port_stdio_transmit((uint8_t*)str, strlen(str));
    
    return (CMD_FN_RET_OK);
}

/*
 * @brief show current mode of operation,
 *           version, and the configuration
 *
 * */
REG_FN(f_stat)
{
    const char * ret = CMD_FN_RET_OK;

    f_decaTDoATag(text, pbss, val);
    f_jstat(text, pbss, val);

    ret = CMD_FN_RET_OK;
    return (ret);
}


REG_FN(f_debug)
{
    debug_en = val? 1 : 0;
    return CMD_FN_RET_OK;
}

#define MAX_NUM_IN_A_LINE 6
REG_FN(f_help_app)
{
    int        indx = 0;
    uint8_t    line_idx = 0;
    const char * ret = NULL;

    char str[MAX_STR_SIZE];
    
    while (known_commands[indx].name != NULL)
    {
        if((line_idx != 0) && (known_commands[indx].fn == NULL))
        {
            sprintf(str,"\r\n");
            port_stdio_transmit((uint8_t*)str, strlen(str));
        }
        
        sprintf(str,"%s\t", known_commands[indx].name);
        port_stdio_transmit((uint8_t*)str, strlen(str));      

        line_idx++;
        if((line_idx == MAX_NUM_IN_A_LINE) || (known_commands[indx].fn == NULL))
        {
            sprintf(str,"\r\n");
            port_stdio_transmit((uint8_t*)str, strlen(str));
            line_idx = 0;
        }

        indx++;
    }

    if(line_idx != 0)
    {
        sprintf(str,"\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }

    ret = CMD_FN_RET_OK;
    return (ret);
}

//REG_FN(f_tcwm)
//{
//    return (CMD_FN_RET_OK);
//}
//
//REG_FN(f_tcfm)
//{
//    return (CMD_FN_RET_OK);
//}
//-----------------------------------------------------------------------------
// Communication change section

/*
 * @brief restore NV configuration from defaults
 *
 * */
REG_FN(f_restore)
{
    port_config_restore_default();
    cmd_uwb_reset();
    return (CMD_FN_RET_OK);
}

/*
 * @brief save configuration
 *
 * */
REG_FN(f_save)
{
    port_config_save();

    return (CMD_FN_RET_OK);
}

//-----------------------------------------------------------------------------

/* end f_xx command functions */

//-----------------------------------------------------------------------------
/* list of known commands:
 * NAME, allowed_MODE,     REG_FN(fn_name)
 * */
static const command_t known_commands[] = {
    /* CMDNAME   MODE   fn     */
    /* Service Commands */
    {"-------- System --------", NULL},    
    {"?", f_help_app},
    {"HELP", f_help_app},
    {"RESTORE", f_restore},
    {"SAVE", f_save},
    //{"DEBUG", f_debug},
    //{"URST", f_prod_urst},
    {"RESET", f_prod_urst}, // name compatible with DWM1001 test
    /*Production Commands */
    {"-------- Operation -----", NULL},    
    {"UOF", f_prod_uof},
    {"UCF", f_prod_ucf},
    {"UCW", f_prod_ucw},
    {"UEE", f_prod_uee},
    {"UEP", f_prod_uep},
    {"UED", f_prod_ued},
    {"URX", f_prod_urx},
    {"USLP",f_prod_sleep},
//    {"UIDLE",f_prod_idle_pll},
//    {"URC", f_prod_idle_rc},
    {"UTWRI", f_prod_utwri},
    {"UTWRR", f_prod_utwrr},
    {"UTX", f_prod_utx},
    //{"USY", f_prod_sync_pin},//todo
    {"-------- Setting -------", NULL},   
    // ======= below are individual commands for production test ========    
    {"CHAN", f_chan},             //!< channel number {5, 9 }
    {"PLEN", f_plen},             //!< 
    {"PAC", f_rxPAC},             //!< 
    {"CODE", f_code},             //!< 
    {"SFD", f_sfd},               //!< New SFD, 0-3
    {"BITR", f_bitRate},         //!< 
    {"PHRM", f_phrMode},          //!< 
    {"PHRR", f_phrRate},          //!< 
    {"STSM", f_stsMode},          //!< 
    {"STSL", f_stsLen},           //!< 
    {"PDOA", f_pdoaMode},         //!<  
    {"UPS", f_prod_ups},
    {"UPPS", f_prod_upps},
    {"URS", f_prod_urs},          // register set
    {"USA", f_prod_usa},
    {"UXT", f_prod_uxt},
    {"UTP", f_prod_utp}, // tx power
    //{"UTS", f_prod_uts}, // considered not necessary?
    {"-------- Read ----------", NULL},    
    {"STAT", f_stat},
    {"UDW", f_prod_udw},
    {"URG", f_prod_urg}, // register get
    {"UGA", f_prod_uga},
    {"UOD", f_prod_uod},
    {"UPG", f_prod_upg},
    {"URD", f_prod_urd},
    {"UTVB", f_prod_utvb},
    {"UTPG", f_prod_utpg},
    {"UHT", f_prod_uht}, //Obtain UWB temperature reading
    {"-------- Write ---------", NULL},  
    {"UOW", f_prod_uow}, // otp write
    {"UOTP",f_prod_uotp},  
    // ======= below are role specific commands for production test ========    // todo: is this necessary?
    {"-------- Work in progress -------", NULL},  
    {"TAG", f_tag},               //!< 
    {"NODE", f_node},             //!< 
    {NULL, NULL}
};
