#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>

#include "vivo_smartpa_cali.h"

/* Mutex to serialize DSP read/write commands*/
static struct mutex routing_lock;

/* vendor ops for speaker calibration */
static struct vivo_cali_ops *local_ops;

#define VIVO_DEV_NAME   "smartpa"
#define VIVO_CTL_IOC_MAGIC  'T'
#define VIVO_IOCTL_SPK_REST  _IOW(VIVO_CTL_IOC_MAGIC, 0x01, int)
#define VIVO_IOCTL_SPK_INTS   _IOR(VIVO_CTL_IOC_MAGIC, 0x02, struct smartpa_msg)
#define VIVO_IOCTL_SPK_INTT  _IOW(VIVO_CTL_IOC_MAGIC, 0x03, int)
#define VIVO_IOCTL_SPK_RFDES 	_IOR(VIVO_CTL_IOC_MAGIC, 0x04, struct smartpa_msg)
#define VIVO_IOCTL_SPK_CHCK _IOR(VIVO_CTL_IOC_MAGIC, 0x05, int)
#define VIVO_IOCTL_SPK_PRARS _IOR(VIVO_CTL_IOC_MAGIC, 0x06, struct smartpa_prars)
#define VIVO_IOCTL_SPK_ADDR  _IOW(VIVO_CTL_IOC_MAGIC, 0x07, unsigned char)
#define VIVO_IOCTL_SPK_MTP_BACKUP _IOR(VIVO_CTL_IOC_MAGIC, 0x08, int)
#define VIVO_IOCTL_SPK_SET   _IOW(VIVO_CTL_IOC_MAGIC, 0x09, struct smartpa_msg)
//vivo linjinping add start for calib fail mute check
#define VIVO_IOCTL_SPK_CALI_FAIL_MUTE  _IOW(VIVO_CTL_IOC_MAGIC, 0x0A, int)
//vivo linjinping add end

extern int mtk_spk_send_ipi_buf_to_dsp(void *data_buffer, uint32_t data_size);
extern int mtk_spk_recv_ipi_buf_from_dsp(int8_t *buffer, int16_t size, uint32_t *buf_len);

static int afe_smartamp_get_set(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
	int32_t ret = 0;
	int32_t rd_length = 0;
	struct mtk_apr apr_buff;
	int32_t i = 0;

	pr_info("[SmartPA:%s] get_set %d param_id %d length %d",
		__func__, get_set, param_id, length);
	if (length > AP_2_DSP_PAYLOAD_SIZE*4) {
		pr_err("[SmartPA:%s] Out of bound length %d", length);
		return -1;
	}

	switch (get_set) {
	case AP_2_DSP_SET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_err("[SmartPA:%s] AP_2_DSP_SET_PARAM param_id %d", __func__, param_id);
			memcpy(apr_buff.data, data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, length+sizeof(param_id));
		}
		break;
	case AP_2_DSP_GET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_info("[SmartPA:%s] AP_2_DSP_GET_PARAM param_id %d", __func__, param_id);
			memset(apr_buff.data, 0, length);
			//update param_id firstly, since param_id can not be sent by get_buf
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(param_id));
			if (ret == 0) {
				ret = mtk_spk_recv_ipi_buf_from_dsp((void *)&apr_buff, length+sizeof(param_id), &rd_length);
				pr_err("[SmartPA:%s] legen-AP_2_DSP_GET rd_length %d, %lld", __func__, rd_length, (unsigned long)(rd_length-sizeof(param_id)));
				if ((ret == 0) && (rd_length <= AP_2_DSP_PAYLOAD_SIZE*4+sizeof(param_id)) && (rd_length >= sizeof(param_id))) {
					memcpy(data_buff, apr_buff.data, rd_length-sizeof(param_id));
				}
			}

			//For Debug
			for (i = 0; i < length/4; i++)
				pr_err("[SmartPA:%s] apr_buff.data[%d] = 0x%0x", __func__, i, apr_buff.data[i]);

			break;
		}
	case AP_2_DSP_SEND_PARAM: /* wangkai add, to pass cali info (struct CALIBRATION_RX_) to DSP */
		{
			apr_buff.param_id = param_id;
			pr_info("[SmartPA:%s] AP_2_DSP_SEND_PARAM param_id %d", __func__, param_id);
			memcpy((void *)&(apr_buff.calib_param), data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(struct mtk_apr));
			if (ret < 0) {
				pr_err("[SmarPA] %s: mtk_spk_send_ipi_buf_to_dsp, param send error!\n", __func__);
			} else {
				pr_info("[SmartPA] %s: mtk_spk_send_ipi_buf_to_dsp, param send successful!\n", __func__);
			}
		}
		break;
	default:
		{
			break;
		}
	}

	return ret;
}

int mtk_afe_smartamp_algo_ctrl(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
	int ret = 0;
	mutex_lock(&routing_lock);
	ret = afe_smartamp_get_set(data_buff, param_id,
		get_set, length);
	mutex_unlock(&routing_lock);
	return ret;
}

static ssize_t vivo_smartpa_debug_read (struct file *file,
	char __user *buf, size_t count, loff_t *offset)
{
	return 0;
}

static ssize_t vivo_smartpa_debug_write (struct file *file,
	const char __user *buf, size_t count, loff_t *offset)
{
	return 0;
}

static long  vivo_smartpa_debug_ioctl (struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int  ret = 0, check = 0;
	struct smartpa_msg msg;
	struct smartpa_prars prars;
	//vivo linjinping add start for calib fail mute check
	int  cali_fail_mute_en = 0;
	//vivo linjinping add end

	memset(&prars, 0, sizeof(struct smartpa_prars));
	memset(&msg, 0, sizeof(struct smartpa_msg));
	switch (cmd) {
	/* Reset MTP */
	case VIVO_IOCTL_SPK_REST:
		printk("[SmartPA]%s, SPK_REST\n", __func__);
		break;
	/* calibrate */
	case VIVO_IOCTL_SPK_INTS:
		printk("[SmartPA]%s, SPK_INTS\n", __func__);
		if (!local_ops || !local_ops->vivo_smartpa_init_dbg) {
			printk("[SmartPA]%s, local_ops or local_ops->vivo_smartpa_init_dbg is NULL\n", __func__);
			msg.msg_result = -1;
		} else {
			check = local_ops->vivo_smartpa_init_dbg(msg.msgs, MSGS_SIZE);
			msg.msg_result = 0;

			//vivo add start by linjinping, operating smartamp.bin file on user layer to solve gki unsymbol problem
			if (local_ops->vivo_smartpa_get_calib_data) {
				msg.msg_result = local_ops->vivo_smartpa_get_calib_data(msg.reserved);
			}
			//vivo add end
		}
		ret = copy_to_user((void *)arg, &msg, sizeof(struct smartpa_msg));
		break;
	case VIVO_IOCTL_SPK_INTT:
		printk("[SmartPA]%s, SPK_INT\n", __func__);
		break;
	case VIVO_IOCTL_SPK_RFDES:
		printk("[SmartPA]%s, SPK_ReadFDes\n", __func__);
		if (!local_ops || !local_ops->vivo_smartpa_read_freq_dbg) {
			printk("[SmartPA] %s, local_ops or local_ops->vivo_smartpa_read_freq_dbg is NULL\n", __func__);
			msg.msg_result = -1;
		} else {
			check = local_ops->vivo_smartpa_read_freq_dbg(msg.msgs, MSGS_SIZE);
			msg.msg_result = 0;
		}
		ret = copy_to_user((void *)arg, &msg, sizeof(struct smartpa_msg));
		break;
	/* checkmtp */
	case VIVO_IOCTL_SPK_CHCK:
		printk("smartpa_ioctl SPK Check MtpEx\n");
		if (!local_ops || !local_ops->vivo_smartpa_check_calib_dbg) {
			printk("[SmartPA]%s, local_ops or local_ops->vivo_smartpa_check_calib_dbg is NULL\n", __func__);
			msg.msg_result = -1;
		} else {
			check = local_ops->vivo_smartpa_check_calib_dbg();
			msg.msg_result = check;
			pr_info("[SmartPA]%s, check %d.\n", __func__, check);
		}
		ret = copy_to_user((__user int *)arg, &check, sizeof(int));
		break;
	case VIVO_IOCTL_SPK_PRARS:
		pr_info("[SmartPA]%s, have not complete it\n", __func__);
		break;
	case VIVO_IOCTL_SPK_ADDR:
		pr_info("[SmartPA]%s, have not complete it\n", __func__);
		break;
	case VIVO_IOCTL_SPK_MTP_BACKUP:
		pr_info("[SmartPA]%s, have not complete it\n", __func__);
		break;

	//vivo add start by linjinping, operating smartamp.bin file on user layer to solve gki unsymbol problem
	case VIVO_IOCTL_SPK_SET:
		ret = copy_from_user(&msg, (void __user *)arg, sizeof(struct smartpa_msg));
		if (ret) {
			pr_info("[SmartPA]%s: Could not copy arg value from user\n", __func__);
			ret = -EFAULT;
			break;
		}

		if (!local_ops || !local_ops->vivo_smartpa_init_calib_data) {
			pr_info("[SmartPA]%s, local_ops or local_ops->vivo_smartpa_init_calib_data is NULL\n", __func__);
			ret = -EFAULT;
		} else {
			ret = local_ops->vivo_smartpa_init_calib_data(msg.msgs, msg.msg_result);
		}

		break;
		//vivo add end

		//vivo linjinping add start for calib fail mute check
	case  VIVO_IOCTL_SPK_CALI_FAIL_MUTE:
		printk("[SmartPA]%s: smartpa_ioctl VIVO_IOCTL_SPK_CALI_FAIL_MUTE\n",  __func__);
		if (!local_ops || !local_ops->vivo_smartpa_cali_fail_mute_enable) {
			printk("[SmartPA]%s, local_ops or local_ops->vivo_smartpa_cali_fail_mute_enable is NULL\n", __func__);
			ret = -EFAULT;
		} else {
			ret = copy_from_user(&cali_fail_mute_en, (void __user *)arg, sizeof(int));
			local_ops->vivo_smartpa_cali_fail_mute_enable(cali_fail_mute_en);
		}

		break;
		//vivo linjinping add end

	default:
		printk("[SmartPA]%s, smartpa Fail IOCTL command no such ioctl cmd = %x\n", __func__,cmd);
		ret = -1;
		break;
    }

    return ret;
}

static long vivo_smartpa_debug_compat_ioctl(
			struct file *file,
			unsigned int cmd,
			unsigned long arg)
{
	return vivo_smartpa_debug_ioctl(file, cmd, arg);
}

static int vivo_smartpa_debug_open(
	struct inode *inode, struct file *file)
{
	printk("[SmartPA]%s, enter\n", __func__);
	return 0;
}

static int vivo_smartpa_debug_release(
	struct inode *inode, struct file *file)
{
	printk("[SmartPA]%s, enter\n", __func__);
	return 0;
}

static const struct file_operations vivo_smartpa_debug_fileops = {
	.owner = THIS_MODULE,
	.open  = vivo_smartpa_debug_open,
	.read  = vivo_smartpa_debug_read,
	.write = vivo_smartpa_debug_write,
	.unlocked_ioctl = vivo_smartpa_debug_ioctl,
	.compat_ioctl = vivo_smartpa_debug_compat_ioctl,
	.release = vivo_smartpa_debug_release,
};

static struct miscdevice vivo_smartpa_debug_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = VIVO_DEV_NAME,
	.fops = &vivo_smartpa_debug_fileops,
};

int vivo_smartpa_debug_probe(struct vivo_cali_ops *ops)
{
	int err = 0;

	printk("%s\n", __func__);

	if (!ops) {
		printk("%s: smartpa_device register failed for not implementing ops\n", __func__);
		return 0;
	}

	local_ops = ops;

	err = misc_register(&vivo_smartpa_debug_device);
	if (err) {
		printk("%s: smartpa_device register failed\n", __func__);
		return err;
	}

	return 0;
}




MODULE_DESCRIPTION("smartpa debug driver");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_LICENSE("GPL");
