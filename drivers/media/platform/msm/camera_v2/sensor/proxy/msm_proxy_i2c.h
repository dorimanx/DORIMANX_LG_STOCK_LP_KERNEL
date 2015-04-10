#ifndef PROXY_I2C_COMMON_H
#define PROXY_I2C_COMMON_H

#include <linux/i2c.h>

//int32_t proxy_i2c_write_table(struct msm_camera_i2c_reg_setting *write_setting);
//int32_t proxy_i2c_write_seq_table(struct msm_camera_i2c_seq_reg_setting *write_setting);
int32_t proxy_i2c_write(uint32_t addr, uint16_t data, enum msm_camera_i2c_data_type data_type);
int32_t proxy_i2c_read(uint32_t addr, uint16_t *data, enum msm_camera_i2c_data_type data_type);
int32_t proxy_i2c_write_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);
int32_t proxy_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);
//int32_t proxy_i2c_e2p_write(uint16_t addr, uint16_t data, enum msm_camera_i2c_data_type data_type);
//int32_t proxy_i2c_e2p_read(uint16_t addr, uint16_t *data, enum msm_camera_i2c_data_type data_type);
//int32_t proxy_i2c_act_write(uint8_t data1, uint8_t data2);

int32_t ProxyWrite16bit( uint32_t RamAddr, uint16_t RamData );
int32_t ProxyRead16bit( uint32_t RamAddr, uint16_t *ReadData );
int32_t ProxyWrite32bit( uint32_t RamAddr, uint32_t RamData );
int32_t ProxyRead32bit(uint32_t RamAddr, uint32_t *ReadData );
int32_t ProxyWrite8bit(uint32_t RegAddr, uint8_t RegData);
int32_t ProxyRead8bit(uint32_t RegAddr, uint8_t *RegData);
//int32_t ProxyE2PRegWriteA(uint16_t RegAddr, uint8_t RegData);
//int32_t ProxyE2PRegReadA(uint16_t RegAddr, uint8_t * RegData);
//int32_t ProxyE2pRed( uint16_t UsAdr, uint8_t UcLen, uint8_t *UcPtr );
//int32_t ProxyE2pWrt( uint16_t UsAdr, uint8_t UcLen, uint8_t *UcPtr );

#define MAX_PROXY_BIN_FILENAME 64
#define MAX_PROXY_BIN_BLOCKS 4
#define MAX_PROXY_BIN_FILES 3

struct proxy_i2c_bin_addr {
	uint16_t bin_str_addr;
	uint16_t bin_end_addr;
	uint16_t reg_str_addr;
};

struct proxy_i2c_bin_entry {
	char  filename[MAX_PROXY_BIN_FILENAME];
	uint32_t filesize;
	uint16_t blocks;
	struct proxy_i2c_bin_addr addrs[MAX_PROXY_BIN_BLOCKS];
};

struct proxy_i2c_bin_list {
	uint16_t files;
	struct proxy_i2c_bin_entry entries[MAX_PROXY_BIN_FILES];
	uint32_t checksum;
};
#endif