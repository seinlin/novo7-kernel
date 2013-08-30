
//#include <linux/module.h>
//#include <linux/kernel.h>
//#include <linux/fs.h>
//#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
/*
#include "../../../arch/arm/plat-s3c64xx/include/plat/gpio-bank-p.h"
#include "../../../arch/arm/plat-s3c64xx/include/plat/regs-gpio.h"
*/

#if 1
#define MAX_ROM_SIZE	(512*1024)
#define MAX_VER_SIZE	0X100
#define MAX_VER_BUF		30
#define MAX_SIZE_PER_TEAM	250

#define TP_CMD_READ_VER	 		0XF4	/*0XF4+Star+Dev_add+R+DATA(1)……DATA(30)*/
#define TP_CMD_WRITE_VER 		0XFC	/*0XFC+CRC1 +DATA(1)……DATA(30)*/
#define TP_CMD_WRITE_LEN_CHKV	0XFB	/*0XFB + ALLLEN=actual len %256 + ALLCHKV*/
#define TP_CMD_START_UPDATE1	0XF1
#define TP_CMD_START_UPDATE2	0XA9
#define TP_CMD_START_RETV		0X21

#define TP_CMD_END_UPDATE1		0XF1
#define TP_CMD_END_UPDATE2		0X52
#define TP_CMD_END_RETV			0X43

#define TP_CMD_READ_STATUS		0XF2

#define TP_STATUS_IDLE			0XA1//0X00
#define TP_STATUS_BUSY			0X00//0XA1
#define TP_STATUS_ERASE_OK		0X13
#define TP_STATUS_ERASE_FAIL	0X1C
#define TP_STATUS_SEND_DATA_OK	0X25
#define TP_STATUS_SEND_DATA_FAIL	0X2C
#define TP_STATUS_CHKV_OK		0X37
#define TP_STATUS_CHKV_FAIL		0X3C
#define TP_STATUS_VER_UPDATE_OK	0X49
#define TP_STATUS_VER_UPDATE_FAIL	0X4C
#define TP_STATUS_ALL_UPDATE_OK	0X50
#define TP_STATUS_ALL_UPDATE_FAIL	0X5C
/*0X00-> means IDLE;*/
/*0XA1-> means BUSY;*/
/*0X13-> means TP ERASE OK; FOR START UPDATE CMD */
/*0X1C-> means TP ERASE FAIL; FOR START UPDATE CMD */
/*0X25-> means TP RECV ONE TEAM DATA(250 BYTE) OK;*/
/*0X2C-> means TP RECV ONE TEAM DATA(250 BYTE) FAIL;*/
/*0X37-> means ALL CHKV OK;*/
/*0X3C-> means ALL CHKV FAIL;*/
/*0X49-> means TP VER BUF UPDATE OK;*/
/*0X4C-> means TP VER BUF UPDATE FAIL;*/
/*0X50-> means ALL UPDATE OK;*/
/*0X5C-> means ALL UPDATE FAIL;*/

#define TP_CMD_WRITE_ONE_TEAM_DATA	0XFF	/*0XFF+Index_num+Length+CRC(N)+DATA(1)+……+DATA(250)*/

#define TP_NEED_REUPDATE		0X15
#define TP_START_CHECK_CRC		0XFA
#define TP_START_CHECK_CRC_RECV	0X95

#define DEBUG_INFO	1
#define LOOPTIMES 	3
#define UPDATE_FAIL_TIMES 1

extern struct i2c_client *infer_ldtp_i2c_client;
int update_status = 0;

static int ldwzic_i2c_rxdata(u8 *cmdbuf,int size)
{
	int ret,loopcon = 0;
	struct i2c_client *client = infer_ldtp_i2c_client;
	
	
	struct i2c_msg msgs[] = {
		{
			.addr 	= client->addr,
			.flags 	= 0,
			.len	= 1,
			.buf	= cmdbuf,
		},
		{
			.addr 	= client->addr,
			.flags	= 1,
			.len 	= size,
			.buf	= cmdbuf,
		},
	};	

	do
	{
		
		ret = i2c_transfer(client->adapter,msgs,2);
		if(ret < 0 )
			msleep(40);
		if(loopcon > LOOPTIMES){
			printk("i2c_transfer error\n");
			return -1;
		}
		loopcon ++;
		
	}while(ret < 0);

	
	return 0;
};

static int ldwzic_i2c_txdata(u8 *cmdbuf,int size)
{
	int ret,loopcon =  0;
	struct i2c_client *client = infer_ldtp_i2c_client;
	
	struct i2c_msg msgs[] = {
		{
			.addr 	= client->addr,
			.flags 	= 0,
			.len	= size,
			.buf	= cmdbuf,
		},
	};
	
	do
	{
		
		ret = i2c_transfer(client->adapter,msgs,1);
		if(ret < 0)
			msleep(40);
		if(loopcon > LOOPTIMES){
			printk("write data error\n");
			return -1;
		}
		loopcon++;
		
	}while(ret < 0);
	
#if DEBUG_INFO
		printk("write data is OK\n");
#endif
	return 0;
}

static int ldwzic_i2c_rxdata2(u8 *cmdbuf,int count1,int count2)
{
	int ret,loopcon = 0;
	struct i2c_client *client = infer_ldtp_i2c_client;
	
	struct i2c_msg msgs[] = {
		{
		.addr 	= client->addr,
			.flags 	= 0,
			.len	= count2,
			.buf	= cmdbuf,
		},
		 {
			.addr 	= client->addr,
			.flags	= 1,
			.len 	= count1,
			.buf	= cmdbuf,
		 },
	};

	do
	{
		ret = i2c_transfer(client->adapter,msgs,2);
		if(ret < 0 )
			msleep(40);
		if(loopcon > LOOPTIMES){
			printk(KERN_EMERG"send 0XF1 and 0XA9 error:");
			return -1;
		}
		loopcon++;
		
	}while(ret < 0);
	
	return 0;
}

/*
extern volatile unsigned long *S3C64XX_GPPCON;
extern volatile unsigned long *S3C64XX_GPPDAT;
extern volatile unsigned long *S3C64XX_GPPPUD;
*/

//used GPIO is GPP11
static void ldwzic_chip_reset(void)
{
/*
#define UGPPCON (*(volatile unsigned long *)S3C64XX_GPPCON)
#define UGPPDAT (*(volatile unsigned long *)S3C64XX_GPPDAT)
#define UGPPPUD (*(volatile unsigned long *)S3C64XX_GPPPUD)

UGPPCON &= 0x3F3FFFFF;
UGPPCON |= 0x400000;

UGPPPUD &= 0x3F3FFFFF;
UGPPPUD |= 0x400000;

UGPPDAT &= 0x77FF;
//UGPPDAT |= 0x800;

msleep(20);

UGPPPUD &= 0x3F3FFFFF;
UGPPPUD |= 0x800000;

UGPPDAT &= 0x77FF;
UGPPDAT |= 0x800;

UGPPPUD &= 0x3F3FFFFF;
//UGPPPUD |= 0x00000;

msleep(50);
*/
}


static int ldwzic_read_status(u8 *status, u8 *result)
{
	int ret;
	u8 cmdbuf[5];
	
	cmdbuf[0]=TP_CMD_READ_STATUS;
	ret = ldwzic_i2c_rxdata(cmdbuf, 2);
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
	}else
	{
	#if DEBUG_INFO
		printk("status: 0x%02X.result=0x%02X\n", cmdbuf[0],cmdbuf[1]);
	#endif
		*status = cmdbuf[0];
		*result = cmdbuf[1];
	}
	return ret;
}

static int ldwzic_cmpVersion(u8 *verfbuf)
{
	int ret = -1,vereq  = -1;
	int j,cont1 = 0;
	#if DEBUG_INFO
	int i;
	#endif
	u8 tpstatus,result;
	u8 bufreadver[MAX_VER_BUF+10];
	int loopcon = 0;
	
	do
	{
		ldwzic_read_status(&tpstatus, &result);
		#if DEBUG_INFO
		printk("TP read staus:0x%02X",tpstatus);
		#endif
		msleep(40);
		if(loopcon++ > 10)
			return -1;
	}while(tpstatus != TP_STATUS_IDLE);
	
for(j=0; j<3;j++)
{
	msleep(40);

	bufreadver[0] = TP_CMD_READ_VER;
	ret = ldwzic_i2c_rxdata(bufreadver, MAX_VER_BUF);
	if (ret < 0) {
	//	ldwzic_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	ret = memcmp(bufreadver,verfbuf,12);
	if(ret != 0)
	{
		if(!cont1)
		{
			j--;
			cont1++;
		}		
	}
	else
	{
		ret = memcmp(bufreadver, verfbuf, MAX_VER_BUF);
		if(ret != 0)
			ret = 1;// not equal
		else
			vereq= 0; //equal
			
		#if DEBUG_INFO// for debug only
		printk("--read ver list-%s-\n",bufreadver);
		for (i=0; i<30; i++)
		{
			printk("%02X,",bufreadver[i]);
			if(i + 1 ==16)
				printk("\n");
		}
		printk("\n--read ver list over--\n");
		#endif
	}
}
	if(vereq == 0)
		return vereq;
	else
		return ret;
}

static int ldwzic_model(u8 *modelbuf,u8 *modelfix)
{
	int ret = -1;
#if DEBUG_INFO
	int i;
#endif
	u8 tpstatus,result;
	u8 bufreadver[MAX_VER_BUF+10];
	int loopcon = 0,retu = -1;
	
	
	do
	{
		ldwzic_read_status(&tpstatus, &result);
#if DEBUG_INFO
		printk("tp read staus:0x%02X",tpstatus);
#endif
		msleep(40);
		if(loopcon++ > 10)
			return -1;
	}while(tpstatus != TP_STATUS_IDLE);
	
	msleep(40);

	bufreadver[0] = TP_CMD_READ_VER;
	ret = ldwzic_i2c_rxdata(bufreadver, MAX_VER_BUF);
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	
	ret = memcmp(bufreadver,modelfix,10);
	if(ret == 0){
		memcpy(modelbuf, &bufreadver[10], 2);
		retu = 1;
	}
#if DEBUG_INFO
	for (i=0; i<30; i++)
	{
		printk("%02X,",bufreadver[i]);
		if(i + 1 ==16)
			printk("\n");
	}
	printk("\n");
#endif
	return retu;
}

static int ldwzic_update_fw(u8 *verfbuf, u8 *fwbuf, int fwlen, int verchkv, int fwchkv)
{
	int updatetimes = 0;
	int updateallTime = 0;
	u8 tpstatus=2,result;
	u8 cmdbuf[MAX_SIZE_PER_TEAM+5];
	int ret,fw_idx,tlen = 0,datalen,i;
	int loopcon = 0,writevertimes = 0;
	int sendloopcon = 0, sendtmp = 0;
	u8 cmdbuf2[MAX_SIZE_PER_TEAM+5];
	
//----------SEND START UPDATE CMD----
	loopcon =0;
sendUpdateCmd:
	updatetimes++;
	if(updatetimes > 2)
	{
		printk("TP update fail.\n");
		return -1;//TP_NEED_REUPDATE;
	}
#if DEBUG_INFO
	printk("TP send start update cmd %d.\n",updatetimes);
#endif
	msleep(40);
	loopcon = 0;
	do
	{
		ldwzic_read_status(&tpstatus, &result);
#if DEBUG_INFO
		printk("TP read staus:0x%02X",tpstatus);
#endif
		msleep(40);
		if(loopcon > LOOPTIMES)
			return -1;
		loopcon++;
	}while(tpstatus != TP_STATUS_IDLE);
	
	cmdbuf[0] = TP_CMD_START_UPDATE1;
	cmdbuf[1] = TP_CMD_START_UPDATE2;
	ret = ldwzic_i2c_rxdata2(cmdbuf, 1, 2);
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	if(cmdbuf[0] != TP_CMD_START_RETV)
	{
	#if DEBUG_INFO
		printk("TP start update cmd recv fail v=%02X\n",cmdbuf[0]);
	#endif
		msleep(1000);
		updateallTime++;
		goto sendUpdateCmd;
	}else{
	#if DEBUG_INFO
		printk("TP start update cmd recv ok.recv=%02X.\n",cmdbuf[0]);
	#endif
		}
	msleep(6000);
	
	do
	{
		ret = ldwzic_read_status(&tpstatus, &result);
		if(ret < 0)
		{
			printk("read status fail!!!\n");
			return ret;
		}
		#if DEBUG_INFO
		printk("TP read start update status =%02X,result=%02X\n",tpstatus,result);
		#endif
		if(tpstatus != TP_STATUS_IDLE)
		{
			msleep(1000);
			updateallTime++;
			if(updateallTime > 11)
			{
				printk("TP resend update cmd %d\n",__LINE__);
				updateallTime = 0;
				goto sendUpdateCmd;
			}else
			{
				printk("TP reread start Update status cmd times=%d\n",updateallTime);
//				goto rereadStartUpadatestatus;
			}
		}else if(result != TP_STATUS_ERASE_OK)
		{
			goto sendUpdateCmd;
		}
		if(loopcon > LOOPTIMES + 10)
			return -1;
		loopcon++;
	}while(tpstatus != TP_STATUS_IDLE);

	cmdbuf2[0] = TP_CMD_START_UPDATE1;
	cmdbuf2[1] = 0x00;
	ret = ldwzic_i2c_rxdata2(cmdbuf2, 1, 2);
#if DEBUG_INFO
	printk("TP repeat update cmd recv ok.recv=%02X.\n",cmdbuf2[0]);
#endif

	
//----------send code len & chkv------------
	printk("TP send code leng &chkv cmd\n");
	msleep(40);
	cmdbuf[0] = TP_CMD_WRITE_LEN_CHKV;
	//cmdbuf[1] = (fwlen >> 8) & 0xff;
	cmdbuf[1] = fwlen  & 0xff;
	cmdbuf[2] = fwchkv & 0xff;
#if DEBUG_INFO
	printk("\nfwlen is :%X	\t fwchkv is :%X \n",cmdbuf[1],cmdbuf[2]);
#endif
	ret = ldwzic_i2c_txdata(cmdbuf, 3);
	if (ret < 0) {
	//	ldwzic_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	msleep(40);
/*	
	loopcon = 0;
	do
	{
		ldwzic_read_status(&tpstatus, &result);
#if DEBUG_INFO
		printk("tp read staus:0x%02X",tpstatus);
#endif
		msleep(40);
		if(loopcon > LOOPTIMES)
			return -1;
		loopcon++;
	}while(tpstatus != TP_STATUS_IDLE);*/
	msleep(100);
//------------start send fw code buf--by per 250 byte ----
	printk("TP start send code.please wait!\n");
	tlen = 0;
	//u8 cmdbuf1[2];
	for(fw_idx=0; tlen <fwlen; fw_idx++,tlen = fw_idx * MAX_SIZE_PER_TEAM)
	{
		cmdbuf[0] = TP_CMD_WRITE_ONE_TEAM_DATA;
		//cmdbuf1[0] = TP_CMD_WRITE_ONE_TEAM_DATA;
		cmdbuf[1] = fw_idx;
		if(fwlen > tlen + MAX_SIZE_PER_TEAM)
			datalen = MAX_SIZE_PER_TEAM;
		else
			datalen = fwlen-tlen;
		#if DEBUG_INFO
		printk("\nTP fw_idx=%d,len = %d\n",fw_idx,datalen);
		#endif
		cmdbuf[3] = 0;
		cmdbuf[2] = datalen;
		
		memcpy(cmdbuf+4, fwbuf+ tlen, datalen);
		for(i=0; i<datalen; i++)
		{
			cmdbuf[3] += cmdbuf[i+4];
		}
		
		//ret = ldwzic_i2c_txdata(cmdbuf, datalen+4);
		ret = ldwzic_i2c_rxdata2(cmdbuf,1,datalen +4);
		if (ret < 0) {
		//	ldwzic_chip_reset();
			printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			return ret;
		}
		#if DEBUG_INFO
		printk("CRC is :%X \n",cmdbuf[3]);
		#endif
		loopcon = 0;
		do
		{
			msleep(40);
			ret = ldwzic_read_status(&tpstatus, &result);
			if(ret < 0)
				return ret;
			if((tpstatus == TP_STATUS_IDLE) && (result != TP_STATUS_SEND_DATA_OK))
			{
				if(fw_idx == sendtmp)
					if(sendloopcon > 10)
						return -1;
					else
						sendloopcon++;
				else{
					sendtmp = fw_idx;
					sendloopcon = 1;
				}
				
				fw_idx--;	
				msleep(40);
			}
			if(loopcon > LOOPTIMES)
				return -1;
			loopcon++;
			
			if(result == TP_STATUS_SEND_DATA_OK)
				msleep(20);
			
		}while(tpstatus != TP_STATUS_IDLE);
	}
	printk("TP send code is OK!\n");
//-----------check crc info-----------
	printk("TP check crc value\n");
	loopcon = 0;
	do
	{
		msleep(100);
		cmdbuf[0] = TP_START_CHECK_CRC;
		ret = ldwzic_i2c_rxdata(cmdbuf, 1);
		if(ret < 0)
			return ret;
		msleep(110);
		if(cmdbuf[0] != TP_START_CHECK_CRC_RECV)
			printk("TP resend check crc cmd.recv=%02X",cmdbuf[0]);
		if(loopcon > LOOPTIMES)
			return -1;
		loopcon ++;
	}while(cmdbuf[0] != TP_START_CHECK_CRC_RECV);

	msleep(2000);
	loopcon = 0;
	do
	{
		msleep(40);
		ret = ldwzic_read_status(&tpstatus, &result);
		if(ret < 0)
		{
			printk("TP commnication fail,exit\n");
			return ret;
		}
		if(tpstatus != TP_STATUS_IDLE)
		{
#if DEBUG_INFO
			printk("TP busy,wait 2S & check again\n");
#endif
			msleep(2000);
		}else
		{
			if(result != TP_STATUS_CHKV_OK)
			{
				printk("tp crc wrong,reupdate again\n");
				goto sendUpdateCmd;
			}
		}
		if(loopcon > LOOPTIMES)
			return -1;
		loopcon++;
	}while(tpstatus != TP_STATUS_IDLE);
	
//---------------write version info--
	 loopcon = 0;
	writevertimes = 0;
	sendtmp = 0;
writeTpVersionInfo:
	printk("TP send write version cmd\n");
	cmdbuf[0] = TP_CMD_WRITE_VER;
	//cmdbuf1[0] = TP_CMD_WRITE_VER;
	memcpy(cmdbuf+2, verfbuf, MAX_VER_BUF);
	cmdbuf[1] = verchkv & 0xFF;
#if DEBUG_INFO
	printk("\n");
	for(i = 0;i < 32; i++){
		printk("%x ,",cmdbuf[i]);
		if(i == 15)
			printk("\n");
	}
	printk("\n");
#endif	
	msleep(40);
	//ret = ldwzic_i2c_txdata(cmdbuf, MAX_VER_BUF+2);
	ret = ldwzic_i2c_rxdata2(cmdbuf,1,MAX_VER_BUF + 2);
	if (ret < 0) {
	//	ldwzic_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	do
	{
		msleep(40);
		ret = ldwzic_read_status(&tpstatus, &result);
		if(ret < 0)
			return ret;
		
		#if DEBUG_INFO
			printk("tpstatus %x\tresult %x\n",tpstatus,result);	
		#endif
		
		if(tpstatus == TP_STATUS_IDLE)
		{
			if(result != TP_STATUS_VER_UPDATE_OK)
			{
				if(writevertimes++ >= 3){
					msleep(40);
					ret = ldwzic_cmpVersion(verfbuf);
					if(ret == 0)
						goto SendUpdateEndCmd;
					else
					{
						
						loopcon ++;
						if(loopcon > LOOPTIMES)
							return -1;
						goto writeTpVersionInfo;
					}
				}
				else
					goto writeTpVersionInfo;
			} 
		}
		else
			if(sendtmp++ > 10)
				return -1;
	}while(tpstatus != TP_STATUS_IDLE);
	sendtmp = 0;
SendUpdateEndCmd:
	loopcon = 0;
	if(sendtmp++ > 3)
		return -1;
	do
	{
		msleep(40);
		printk("TP send update end cmd\n");
		cmdbuf[0] = TP_CMD_END_UPDATE1;
		cmdbuf[1] = TP_CMD_END_UPDATE2;
		ret = ldwzic_i2c_rxdata2(cmdbuf, 1, 2);
		if (ret < 0) {
		//	ldwzic_chip_reset();
			printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			return ret;
		}
		msleep(40);
		if(loopcon > LOOPTIMES)
			return -1;
		loopcon++;
	}while(cmdbuf[0] != TP_CMD_END_RETV);
	loopcon = 0;
	do
	{
		msleep(40);
		ret = ldwzic_read_status(&tpstatus, &result);
		if (ret < 0) {
		//	ldwzic_chip_reset();
			printk("%s read_data i2c_rxdata failed: %d,line=%d.\n", __func__, ret,__LINE__);
			return ret;
		}
		
		if(tpstatus == TP_STATUS_BUSY){
			if(result == TP_STATUS_ALL_UPDATE_OK)
				return 0;
			else if(result == TP_STATUS_ALL_UPDATE_FAIL)
				goto SendUpdateEndCmd;
			}
		if(loopcon > LOOPTIMES)
			return -1;
		loopcon++;
	}while(tpstatus != TP_STATUS_IDLE);
	
	if(TP_STATUS_ALL_UPDATE_OK == result)
	{
		printk("\nTP update ok!!!\n");
		return 0;
	}else
	{
	//	ldwzic_chip_reset();
	//	return TP_NEED_REUPDATE;
		goto SendUpdateEndCmd;
	}
		
}

#ifdef CONFIG_IPS_BD7248F_800x480_TOUCHSCREEN
#include "IPS_update_ori_fw_6201.hex"
#else
#include "IPS_update_ori_fw_6301.hex"
#endif
int __init Upgrade_FW_CFG(void)
{
	u8 model[2];
	unsigned int verchkv = 0;
	unsigned int codechkv = 0;
	int i,mod = 0;
	int verflen=0,codeflen=0;
	int updatefailtimes = 0;
	//char strbuf[100];
	//char verBuf[MAX_VER_BUF];
	int ret,selete_model1 = 0,selete_model2 = 0;
	const char *ver_buf = NULL,*code_buf = NULL;
#ifdef CONFIG_IPS_BD7248F_800x480_TOUCHSCREEN
	u8 modelfix[] = {0x42, 0x4E, 0x30, 0x37, 0x30, 0x36, 0x32, 0x30, 0x31, 0x2D};
#else
	u8 modelfix[] = {0x42, 0x4E, 0x30, 0x37, 0x30, 0x36, 0x33, 0x30, 0x31, 0x2D};
#endif	

#if DEBUG_INFO
	for (i=0; i<10; i++)
	{
		printk("%02X,",modelfix[i]);
	}
	printk("\n");
#endif


	ldwzic_chip_reset();

//--------------selecting code-----------
	while(mod <3){
		msleep(40);
		ret = ldwzic_model(model,modelfix);
		if(ret < 0)
			printk("%d times read model is failed!",mod);
		else
		{
#if DEBUG_INFO
			printk("model[0] %x\t model[1] %x\n",model[0],model[1]);
#endif
			if(0x30 == model[0] && 0x30 == model[1])
				selete_model1++;
				
			else if(0x31 == model[0] && 0x31 == model[1])
				selete_model2++;
		}
		mod++;
	}

#ifdef CONFIG_IPS_BD7248F_800x480_TOUCHSCREEN	//标清6201
	printk("select TP model 6201\n");
	if(selete_model1 >= 2){
		//20120504
		ver_buf = ldtpversion6201_00;
		code_buf = ldtpcode6201_00;

		verflen = sizeof(ldtpversion6201_00);
		codeflen = sizeof(ldtpcode6201_00);
		}
	/*
	else if(selete_model2 >= 2){
		//
		ver_buf = ldtpversion6201_11;
		code_buf = ldtpcode6201_11;

		verflen = sizeof(ldtpversion6201_11);
		codeflen = sizeof(ldtpcode6201_11);
		}*/
	else{
		update_status = 0;
		printk("model is inexistence!");
		goto updatefail;
	}/*
		update_status = 0;
		goto updatefail;*/
#else										//高清6301
	printk("select TP model 6301\n");
		if(selete_model1 >= 2){
		//20120510
		ver_buf = ldtpversion6301_00;
		code_buf = ldtpcode6301_00;

		verflen = sizeof(ldtpversion6301_00);
		codeflen = sizeof(ldtpcode6301_00);
		}
	/*else  if(selete_model2 >= 2){
		//
		ver_buf = ldtpversion6301_11;
		code_buf = ldtpcode6301_11;

		verflen = sizeof(ldtpversion6301_11);
		codeflen = sizeof(ldtpcode6301_11);
		}*/
	else{
		update_status = 0;
		printk("model is inexistence!");
		goto updatefail;
	}
	/*
	update_status = 0;
	goto updateend;
	*/
#endif

	for(i=0; i< verflen; i++)
	{
		verchkv += ver_buf[i];
	}
	for(i=0; i< codeflen; i++)
	{
		codechkv += code_buf[i];
	}
//--------------------------------------------------------------------
//-----chk tp code version here 1st
tpreupdate:
		ret = ldwzic_cmpVersion((u8 *)ver_buf);
		if(ret == 0)
		{
			printk( "TP version equal,no need update\n");
			update_status =0;
			goto updateend;
		}
		
		if(ret < 0)
		{
			printk( "TP version read fail\n");
			update_status = 0;
			goto updatefail;
		}
		else
		{
			printk("TP version not equal,need update,6201 and 6301 version 5.17 9:00\n");
/*			sprintf(strbuf, "version not equal,need update\n");
			log_ret = log_fd->f_op->write(log_fd, strbuf, strlen(strbuf) ,&log_fd->f_pos);
			if (log_ret < 0)
				printk("tpic write log file failed,%d.\n",__LINE__);
*/
				
			ret = ldwzic_update_fw((u8 *)ver_buf, (u8 *)code_buf, codeflen, verchkv, codechkv);
			if(ret < 0)
			{
				
/*				sprintf(strbuf,"need reupdate\n");
				log_ret = log_fd->f_op->write(log_fd, strbuf, strlen(strbuf) ,&log_fd->f_pos);
				if (log_ret < 0)
					printk("tpic write log file failed,%d.\n",__LINE__);
*/				if(updatefailtimes < UPDATE_FAIL_TIMES){
					printk("TP need reupdate\n");
					msleep(40);
					ldwzic_chip_reset();
					updatefailtimes++;
					goto tpreupdate;
				}
				else{
					update_status = 2;
					printk("TP update fail,exit update\n");
				}
			}else if(ret > 0)
			{
				update_status = 2;
				printk("TP communication fail,exit update\n");
			}
			else
				update_status = 1;
		}
		ldwzic_chip_reset();
//--------------------------------------------------------------------
updatefail:
updateend:
#if DEBUG_INFO
	printk("TP update_status is: %d\n",update_status);
#endif
	return 1;

}

#endif

