/*
 * Driver for the PCM512x CODECs
 *
 * Author:	Mark Brown <broonie@linaro.org>
 *		Copyright 2014 Linaro Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <plat/sys_config.h>
#include <linux/err.h>

#include "pcm512x.h"


#define  CODEC_NAME "pcm5122"

/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

static __u32 twi_id = 0;


/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CODEC_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CODEC_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}

/*
** Be used to para for the codec from the fex file
**
*/
static int codec_fetch_sysconfig_para(void)
{
    char name[I2C_NAME_SIZE];
    __u32    codec_used = -1;
    __u32 twi_addr = 0;
    int ret = -1;
    
    script_parser_value_type_t type = SCRIPT_PARSER_VALUE_TYPE_STRING;

    if(SCRIPT_PARSER_OK != script_parser_fetch("codec_para", "codec_used", &codec_used, 1)){
        pr_err("%s: script_parser_fetch err. \n", __func__);
            goto script_parser_fetch_err;
        }
        if(1 != codec_used){
            pr_err("%s: codec_used. \n",  __func__);
                return ret;
        }
        
 	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("codec_para", "codec_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(strcmp(CODEC_NAME, name)){
		pr_err("%s: name %s does not match CODEC_NAME. \n", __func__, name);
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch("codec_para", "codec_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	pr_info("%s: after: codec_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("codec_para", "codec_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	pr_info("%s: codec_twi_id is %d. \n", __func__, twi_id);   

        return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
    
}
static int pcm512x_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct regmap_config config = pcm512x_regmap;

	/* msb needs to be set to enable auto-increment of addresses */
	config.read_flag_mask = 0x80;
	config.write_flag_mask = 0x80;

	regmap = devm_regmap_init_i2c(i2c, &config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return pcm512x_probe(&i2c->dev, regmap);
}

static int pcm512x_i2c_remove(struct i2c_client *i2c)
{
	pcm512x_remove(&i2c->dev);
	return 0;
}

static const struct i2c_device_id pcm512x_i2c_id[] = {
	{ "pcm5121", },
	{ "pcm5122", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcm512x_i2c_id);

static const struct of_device_id pcm512x_of_match[] = {
	{ .compatible = "ti,pcm5121", },
	{ .compatible = "ti,pcm5122", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm512x_of_match);

static struct i2c_driver pcm512x_i2c_driver = {
       .class  = I2C_CLASS_HWMON,                   /*class of slave device*/
	.probe 		= pcm512x_i2c_probe,
	.remove 	= pcm512x_i2c_remove,
	.detect    = ctp_detect,
	.id_table	= pcm512x_i2c_id,
	.driver		= {
		.name	= "pcm512x",
		.of_match_table = pcm512x_of_match,
		.pm     = &pcm512x_pm_ops,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};

static int __init pcm512x_init(void)
{
 codec_fetch_sysconfig_para();
    
    i2c_add_driver(&pcm512x_i2c_driver);
    return 0;
}

static void __exit pcm512x_exit(void)
{
    i2c_del_driver(&pcm512x_i2c_driver);
    
}

module_init(pcm512x_init);
module_exit(pcm512x_exit);

MODULE_DESCRIPTION("ASoC PCM512x codec driver - I2C");
MODULE_AUTHOR("Mark Brown <broonie@linaro.org> & Peter Chen <peter.chen@lemaker.org>");
MODULE_LICENSE("GPL v2");
