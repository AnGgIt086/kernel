
#define DEBUG
#define pr_fmt(fmt) "lra_drv_core: %s:" fmt, __func__


#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mman.h>

#include "vivo_haptic_core.h"

static DEFINE_MUTEX(haptic_lock);
static LIST_HEAD(haptic_list);
static DEFINE_SPINLOCK(haptic_list_lock);

static int vibrator_count = 0;
static struct miscdevice haptic_core_misc;
static struct haptic_core_dev *gchip = NULL;


#define INVALID_VALUE -1

static int find_first_empty_owner(struct file *array[], int max_len)
{
	int id = 0;

	for (id = 0; id < max_len; id++)
		if (!array[id])
			break;

	if (id >= max_len)
		return -ENOSPC;

	return id;
}

static inline bool user_id_is_valid(int id)
{
	return (id >=0) && (id < DYNAMIC_USER_IDS);
}

static int haptic_effect_from_user(struct haptic_effect *event, const char __user *buffer)
{
	if (copy_from_user(event, buffer, sizeof(struct haptic_effect)))
		return -EFAULT;

	return 0;
}

static int haptic_write_event_from_user(struct haptic_write_event *event, const char __user *buffer)
{
	if (copy_from_user(event, buffer, sizeof(struct haptic_write_event)))
		return -EFAULT;

	return 0;
}

static int check_effect_access(struct haptic_misc *hap_misc_dev, int user_id, struct file *file)
{

	if (hap_misc_dev->user_status.channel_flag != UPLOAD) {
		pr_err("Now is AAC channel\n");
		return -EACCES;
	}

	if (hap_misc_dev->user_status.current_owner != file) {
		pr_err("Not belong to current effect\n");
		return -EACCES;
	}

	if (!user_id_is_valid(user_id)) {
		pr_err("Not valid id\n");
		return -EFAULT;
	} else {
		if (!hap_misc_dev->effect_owners[user_id]) {
			pr_err("Not valid status\n");
			return -EINVAL;
		}
	}

	if (user_id != hap_misc_dev->user_status.user_id) {
		pr_err("Not support last effect\n");
		return -EPERM;
	}

	return 0;
}

/*******************************************************************************************
 *
 * haptic fops
 *
 *******************************************************************************************/
static int haptic_open(struct inode *inode, struct file *file)
{
	struct haptic_misc *hap_misc_dev = NULL;
	struct haptic_handle  *handle = NULL;

	file->private_data = NULL;

	spin_lock(&haptic_list_lock);
	list_for_each_entry(hap_misc_dev, &haptic_list, list) {
		if (hap_misc_dev->devt == inode->i_rdev) {
			pr_info("dev_t_hap=%#x, dev_t_inode=%#x, dev struct found\n", hap_misc_dev->devt, inode->i_rdev);
			file->private_data = hap_misc_dev;
			handle = hap_misc_dev->handle;
			break;
		}
	}
	spin_unlock(&haptic_list_lock);


	if (!(file->private_data) || !(handle)) {
		pr_err("no device found or handle operation not implement\n");
		return -ENODEV;
	}

	pr_info("hap miscdev open count = %d\n", hap_misc_dev->open);

	if (!hap_misc_dev->open) {
		// You can do initlization when first open device
		if (handle->init_dev) {
			handle->init_dev(handle);
		}
		hap_misc_dev->open++;
	}

	return 0;
}

static ssize_t haptic_read(struct file *file, char __user *buff, size_t len, loff_t *offset)
{
	return len;
}

/* 用于加载效果和开始播放振动 */
static ssize_t haptic_write(struct file *file, const char __user *buff, size_t len, loff_t *off)
{

	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_handle *hh = hap_misc_dev->handle;
	int ret = 0;
	struct haptic_write_event event;

	ret = mutex_lock_interruptible(&hap_misc_dev->lock);
	if (ret) {
		pr_err("lock not correct, (%px)\n", file);
		return ret;
	}

	if (haptic_write_event_from_user(&event, buff)) {
		pr_err("Copy form user error, (%px)\n", file);
		mutex_unlock(&hap_misc_dev->lock);
		return -EFAULT;
	}

	switch (event.type) {

	case HAPTIC_PLAY_EVENT:
		pr_info("Playback on, len=%d (%px)\n", len, file);

		ret = check_effect_access(hap_misc_dev, event.code, file);
		if (ret) {
			mutex_unlock(&hap_misc_dev->lock);
			return -EACCES;
		}

		ret = hh->playback(hh, 1);
		if (ret < 0) {
			pr_err("Playback failed, ret=%d\n", ret);
			mutex_unlock(&hap_misc_dev->lock);
			return -EFAULT;
		}
		break;

	case HAPTIC_GAIN_EVENT:
		pr_info("Gain %d, len=%d (%px)\n", event.value, len, file);
		hh->set_gain(hh, event.value);
		break;

	default:
		pr_err("Invalid event type, len=%d (%px)\n", len, file);
	}

	mutex_unlock(&hap_misc_dev->lock);

	return len;
}

static int haptic_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	return 0;
}

static long haptic_do_ioctl(struct file *file, unsigned int cmd, void __user *p, int compat_mode)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_handle *hh = hap_misc_dev->handle;
	struct haptic_effect effect;

	uint8_t __user *ip = (uint8_t __user *)p;
	int user_id = (int)(unsigned long)p;
	int ret = 0;
	int id = 0;
	int effectNo = 0;
	int motorType = 619;
	int f0 = 170;
	enum ic_type driver_ic = VIB_NONE;
	struct haptic_cali_param cali_param_list = { 0 };

	switch (cmd) {

	case HAPTIC_UPLOAD:
	{
		pr_info("Upload cmd=%x (%px)\n", cmd, file);
		if (haptic_effect_from_user(&effect, p)) {
			pr_err("Upload copy not correct\n");
			return -EFAULT;
		}

		if (hap_misc_dev->user_status.channel_flag == AAC_STREAM_MODE) {
			pr_err("upload insert aac stop\n");
			if (hh->rtp_stop)
				hh->rtp_stop(hh);
		} else {

			if (((hap_misc_dev->user_status.current_owner != file) &&
					(hap_misc_dev->effect_owners[hap_misc_dev->user_status.user_id]))
				|| effect.id != INVALID_VALUE) {

				pr_info("Upload insert cancel, user id (%d)\n", effect.id);
				ret = hh->playback(hh, 0);
				if (ret < 0) {
					pr_err("Insert stop failed, ret=%d\n", ret);
					return -EFAULT;
				}

				if (hh->erase) {
					ret = hh->erase(hh);
					if (ret < 0) {
						pr_err("Insert erase play failed, ret=%d\n", ret);
						return -EFAULT;
					}
				}
			}
		}

		if (effect.id == INVALID_VALUE) {

			id = find_first_empty_owner(hap_misc_dev->effect_owners, DYNAMIC_USER_IDS);
			if (id < 0) {
				pr_err("No free bit to use\n");
				return -ENOSPC;
			}
			effect.id = id;

		} else {
			pr_info("Effect userid over (%d)\n", effect.id);
			if (user_id_is_valid(effect.id))
				id = effect.id;
			else {
				pr_err("user id is over range, %d\n", effect.id);
				return -EFAULT;
			}
		}

		ret = hh->upload(hh, &effect);
		if (ret < 0) {
			pr_err("upload effect failed, ret=%d\n", ret);
			return -EFAULT;
		}

		hap_misc_dev->effect_owners[id] = file;
		hap_misc_dev->user_status.current_owner = file;
		hap_misc_dev->user_status.user_id = id;
		hap_misc_dev->user_status.channel_flag = UPLOAD;

		if (put_user(effect.id, &(((struct haptic_effect __user *)p)->id))) {
			pr_err("Upload put user not correct\n");
			return -EFAULT;
		}

		break;
	}

	case HAPTIC_STOP: //停止振动
	{
		pr_info("Stop id=%d\n (%px)", user_id, file);

		ret = check_effect_access(hap_misc_dev, user_id, file);
		if (ret) {
			if (user_id_is_valid(user_id)) {
				if (hap_misc_dev->effect_owners[user_id] == file)
					hap_misc_dev->effect_owners[user_id] = NULL;
			}
			return 0;
		}

		if (user_id_is_valid(user_id))
			hap_misc_dev->effect_owners[user_id] = NULL;

		ret = hh->playback(hh, 0);
		if (ret < 0) {
			pr_err("Stop failed, ret=%d\n", ret);
			return -EFAULT;
		}

		if (hh->erase) {
			ret = hh->erase(hh);
			if (ret < 0) {
				pr_err("erase play failed, ret=%d\n", ret);
				return -EFAULT;
			}
		}
		break;
	}

	case HAPTIC_SUPPORT_BITMASK:
		pr_info("Feedback support features\n");
		put_user(hh->hap_bit[0], ip);
		break;

	case HAPTIC_TRIGGER_INTENSITY:
		pr_info("Set trigger intensity\n");
		if (!hh->set_trigger_intensity)
			return -EPERM;
		hh->set_trigger_intensity(hh, user_id);
		break;

	case HAPTIC_DISPATCH_CALI_PARAM:
		if (copy_from_user(&cali_param_list, p, sizeof(struct haptic_cali_param))) {
			pr_err("Get cali param failed\n");
			return -EFAULT;
		}

		if (hh->set_cali_params)
			hh->set_cali_params(hh, &cali_param_list);
		break;

	case HAPTIC_JUDGE_EFFECT_SUPPORT:
		if (hh->judge_effect_support == NULL)
			return -EFAULT;
		if (copy_from_user(&effectNo, ip, sizeof(int))) {
			pr_err("haptic judge effect support copy from failed\n");
			return -EFAULT;
		}
		ret = hh->judge_effect_support(hh, effectNo);
		if (ret < 0) {
			pr_err("judge: no effect, ret=%d\n", ret);
			ret = 0;
		} else {
			ret = 1;
		}
		effectNo = ret;
		if (copy_to_user(ip, &effectNo, sizeof(int))) {
			pr_err("haptic judge effect support copy failed\n");
			return -EFAULT;
		}
		break;

	case HAPTIC_GET_MOTOR_TYPE:
		if (hh->get_motor_type)
			hh->get_motor_type(hh, &motorType);
		pr_err("motorType = %d\n", motorType);
		if (copy_to_user(ip, &motorType, sizeof(int))) {
			pr_err("haptic get motor type failed\n");
			return -EFAULT;
		}
		break;

	case HAPTIC_GET_F0: // Get actual motor f0, not theoretical value
		if (hh->get_f0)
			hh->get_f0(hh, &f0);
		pr_info("motor f0 = %d\n", f0);
		if (copy_to_user(ip, &f0, sizeof(int))) {
			pr_err("haptic get f0 copy failed\n");
			return -EFAULT;
		}
		break;

	case HAPTIC_GET_DRIVER_IC_TYPE:
		if (hh->get_driver_ic)
			hh->get_driver_ic(hh, &driver_ic);
		pr_info("driver_ic = %d\n", driver_ic);
		if (copy_to_user(ip, &driver_ic, sizeof(int))) {
			pr_err("haptic get driver ic type copy failed\n");
			return -EFAULT;
		}
		break;
/* only used by hal aac richtap so begin */
/*****************************************/

	case RICHTAP_GET_HWINFO:
		{
			uint8_t tmp = 0x05;
			if (hh->get_ic_identity_NO)
				hh->get_ic_identity_NO(hh, &tmp);
			if (copy_to_user((void __user *)p, &tmp, sizeof(uint8_t)))
				ret = -EFAULT;
			break;
		}

	case RICHTAP_GET_F0:
		{
			int tmp = 170;
			if (hh->get_motor_f0)
				hh->get_motor_f0(hh, &tmp);
			if (copy_to_user((void __user *)p, &tmp, sizeof(uint32_t)))
				ret = -EFAULT;
			break;
		}

	case RICHTAP_SETTING_GAIN:
		{
			int tmp;
			tmp = (int)(unsigned long)p;
			if (tmp > 0x80)
				tmp = 0x80;
			if (hh->set_gain_reg_direct)
				hh->set_gain_reg_direct(hh, tmp);
			break;
		}
	case RICHTAP_RTP_MODE:

		if ((hap_misc_dev->effect_owners[hap_misc_dev->user_status.user_id]) && (hap_misc_dev->user_status.channel_flag == UPLOAD)) {
			pr_info("RICHTAP_RTP_MODE insert cancel\n");
			ret = hh->playback(hh, 0);
			if (ret < 0) {
				pr_err("Stop failed, ret=%d\n", ret);
				return -EFAULT;
			}

			if (hh->erase) {
				ret = hh->erase(hh);
				if (ret < 0) {
					pr_err("erase play failed, ret=%d\n", ret);
					return -EFAULT;
				}
			}

		}

		if (hh->play_rtp_less_than_FIFO)
			hh->play_rtp_less_than_FIFO(hh, p);
		hap_misc_dev->user_status.channel_flag = AAC_RTP_MODE;

		break;

	case RICHTAP_STREAM_MODE:

		if ((hap_misc_dev->effect_owners[hap_misc_dev->user_status.user_id]) && (hap_misc_dev->user_status.channel_flag == UPLOAD)) {
			pr_info("RICHTAP_STREAM_MODE insert cancel\n");
			ret = hh->playback(hh, 0);
			if (ret < 0) {
				pr_err("Stop failed, ret=%d\n", ret);
				return -EFAULT;
			}

			if (hh->erase) {
				ret = hh->erase(hh);
				if (ret < 0) {
					pr_err("erase play failed, ret=%d\n", ret);
					return -EFAULT;
				}
			}

		}

		if (hh->play_rtp_from_cycle_buffer)
			hh->play_rtp_from_cycle_buffer(hh);
		hap_misc_dev->user_status.channel_flag = AAC_STREAM_MODE;
		break;

	case RICHTAP_STOP_MODE:
		if (hh->rtp_stop)
			hh->rtp_stop(hh);
		break;

	case RICHTAP_OFF_MODE:
		break;

/*****************************************/
/* only used by hal aac richtap so end */

	default:
		pr_info("unknown cmd, cmd (0x%x)\n", cmd);
		break;

	}

	return ret;
}

static long haptic_ioctl_handler(struct file *file, unsigned int cmd, void __user *p, int compat_mode)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	int retval;

	retval = mutex_lock_interruptible(&hap_misc_dev->lock);
	if (retval)
		return retval;

	retval = haptic_do_ioctl(file, cmd, p, compat_mode);

	mutex_unlock(&hap_misc_dev->lock);
	return retval;
}

static long haptic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return haptic_ioctl_handler(file, cmd, (void __user *)arg, 0);
}

#ifdef CONFIG_COMPAT
static long haptic_ioctl_compat(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_handle *hh = hap_misc_dev->handle;
	int ret = 0;


	pr_debug("32bit compat mode, to be deal\n");

	// stop vibrator direct

	ret = mutex_lock_interruptible(&hap_misc_dev->lock);
	if (ret)
		return ret;

	ret = hh->playback(hh, 0);
	if (ret < 0) {
		pr_err("Stop failed, ret=%d\n", ret);
		mutex_unlock(&hap_misc_dev->lock);
		return -EFAULT;
	}

	if (hh->erase) {
		ret = hh->erase(hh);
		if (ret < 0) {
			pr_err("erase play failed, ret=%d\n", ret);
			mutex_unlock(&hap_misc_dev->lock);
			return -EFAULT;
		}
	}

	mutex_unlock(&hap_misc_dev->lock);

	return -EPERM;
	//return haptic_ioctl_handler(file, cmd, compat_ptr(arg), 1);
}
#endif

static int haptic_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long phys;
//	int order;
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_handle *hh = hap_misc_dev->handle;
	int ret = -EINVAL;

	if (test_bit(HAPTIC_MASK_BIT_AAC_RICHTAP, hh->hap_bit)) {
		if (hh->start_buf) {
//			#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 7, 0)
//				//only accept PROT_READ, PROT_WRITE and MAP_SHARED from the API of mmap
//				vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0) | calc_vm_flag_bits(MAP_SHARED);
//				vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;
//				if (vma && (pgprot_val(vma->vm_page_prot) != pgprot_val(vm_get_page_prot(vm_flags))))
//					return -EPERM;
//
//				if (vma && ((vma->vm_end - vma->vm_start) != (PAGE_SIZE << RICHTAP_MMAP_PAGE_ORDER)))
//					return -ENOMEM;
//			#endif
				phys = virt_to_phys(hh->start_buf);

				ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT), (vma->vm_end - vma->vm_start), vma->vm_page_prot);
				if (ret) {
					pr_err("Error aac mmap failed\n");
					return ret;
				}
				pr_info("aac richtap mmap successed\n");
				ret = 0;
		} else
			pr_err("start buf null\n");

	} else {
		if (hh->haptic_container) {
			phys = virt_to_phys(hh->haptic_container);

			ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT), (vma->vm_end - vma->vm_start), vma->vm_page_prot);
			if (ret) {
				pr_err("Error vivo mmap failed\n");
				return ret;
			}
			pr_err("vivo mmap successed\n");
			ret = 0;
		} else
			pr_err("haptic container null\n");
	}

	return ret;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = haptic_open,
	.read = haptic_read,
	.write = haptic_write,
	.unlocked_ioctl = haptic_ioctl,
	.mmap = haptic_mmap,

#ifdef CONFIG_COMPAT
	.compat_ioctl = haptic_ioctl_compat,
#endif

	.release = haptic_release,
};


/*****************************************************************************************************************
 *
 * LogSystem exception buried point.
 * Note: Results are only valid if reported every 10 minutes, and those reported within 10 minutes are discarded
 *
 *****************************************************************************************************************/
#ifdef VIVO_HAPTIC_VCODE

extern int writeData(char *modelId,  char *filename, char *data);

static int vcode_haptic_init(struct vcode_infomation_statistic *vcode)
{
	// 初始统计上报策略, count_strategy暂未使用
	vcode->count_strategy[HBOOST_TIMEOUT] = ONLY_THREE_TIMES_OCCUR;
	vcode->count_strategy[BOOT_IMPEDANCE_CHECK_ERROR] = ONLY_ONCE_FIRST_OCCUR;
	vcode->count_strategy[AUTO_STANDBY_ERROR] = EVERY_TIMES_COUNT;
	vcode->count_strategy[SCENE_NOT_EXIST] = EVERY_TIMES_COUNT;
	vcode->count_strategy[EFFECT_NOT_EXIST] = EVERY_TIMES_COUNT;
	vcode->count_strategy[BIN_FILE_LOAD_ERROR] = EVERY_TIMES_COUNT;
	vcode->count_strategy[RTP_MEM_ALLOC_ERROR] = EVERY_TIMES_COUNT;
	vcode->count_strategy[CONSTANT_MODE_VOL_ERROR] = EVERY_TIMES_COUNT;

	vcode->log_strategy[HBOOST_TIMEOUT] = KMSG_SAVE_ONCE;
	vcode->log_strategy[BOOT_IMPEDANCE_CHECK_ERROR] = KMSG_SAVE_ONCE;
	vcode->log_strategy[AUTO_STANDBY_ERROR] = KMSG_SAVE_EVERYTIME;
	vcode->log_strategy[SCENE_NOT_EXIST] = SELF_DEFINE;
	vcode->log_strategy[EFFECT_NOT_EXIST] = SELF_DEFINE;
	vcode->log_strategy[BIN_FILE_LOAD_ERROR] = SELF_DEFINE;
	vcode->log_strategy[RTP_MEM_ALLOC_ERROR] = SELF_DEFINE;
	vcode->log_strategy[CONSTANT_MODE_VOL_ERROR] = SELF_DEFINE;

	return 0;
}

//[{"moduleid":"5130","eventId":"00097|012","ic_type":"15","fullhash":"11","logpath":"dev/kmsg","extype":"0","subtype":"1_2_4_5_6",”times”:”5_4_2_5_6”,"exInfo":"abcd"}]
#define SUB_BUF_LEN 128

static void vcode_haptic_upload_handler(void)
{
	int i = 0;
	//int sub_ex_count = 0;
	bool save_kmsg_flag = false;
	bool first_data_flag = false;
	char sub_type[SUB_BUF_LEN] = { 0 };
	char sub_times[SUB_BUF_LEN] = { 0 };
	char upload_buffer[ENTRY_LEN_MAX] = { "[{\"moduleid\":\"5130\",\"eventId\":\"00097|012\",\"ic_type\":\"15\"," }; // HEAD
	char log_tag[] = { "\"fullhash\":\"11\",\"logpath\":\"dev/kmsg\"," };
	char tail_buffer[] = { "\"}]" };
	char exInfo_buffer[ENTRY_LEN_MAX] = { 0 };
	int type_len = 0, times_len = 0, exInfo_len = 0;

	struct vcode_infomation_statistic *vcode = NULL;

	vcode = &gchip->vcode;

	type_len += snprintf(sub_type + type_len, SUB_BUF_LEN - type_len, "\"extype\":\"1\",\"subtype\":\"");

	times_len += snprintf(sub_times + times_len, SUB_BUF_LEN - times_len, "\"times\":\"");
	for (i = 0; i < HAPTIC_VCODE_ALL; i++) {
		if (vcode->times_statistic[i] > 0) {
			if ((vcode->log_strategy[i] == KMSG_SAVE_ONCE) && (!test_bit(i, vcode->first_upload_bitmask)))
				save_kmsg_flag = true;
			if (vcode->log_strategy[i] == KMSG_SAVE_EVERYTIME)
				save_kmsg_flag = true;
			if (!first_data_flag) {
				type_len += snprintf(sub_type + type_len, SUB_BUF_LEN - type_len, "%d", i);
				times_len += snprintf(sub_times + times_len, SUB_BUF_LEN - times_len, "%d", vcode->times_statistic[i]);
				first_data_flag = true;
			} else {
				type_len += snprintf(sub_type + type_len, SUB_BUF_LEN - type_len, "_%d", i);
				times_len += snprintf(sub_times + times_len, SUB_BUF_LEN - times_len, "_%d", vcode->times_statistic[i]);
			}
			vcode->times_statistic[i] = 0;
			set_bit(i, vcode->first_upload_bitmask);
		}
	}

	type_len += snprintf(sub_type + type_len, SUB_BUF_LEN - type_len, "\",");
	times_len += snprintf(sub_times + times_len, SUB_BUF_LEN - times_len, "\",");

	// combine upload data
	if (save_kmsg_flag) {
		strncat(upload_buffer, log_tag, sizeof(upload_buffer) - strlen(upload_buffer) - 1);
	}

	strncat(upload_buffer, sub_type, sizeof(upload_buffer) - strlen(upload_buffer) - 1);
	strncat(upload_buffer, sub_times, sizeof(upload_buffer) - strlen(upload_buffer) - 1);
	exInfo_len += snprintf(exInfo_buffer + exInfo_len, ENTRY_LEN_MAX - strlen(upload_buffer) - strlen(tail_buffer) - 1 - exInfo_len, "\"exInfo\":\"" "%s", vcode->exinfo);
	strncat(upload_buffer, exInfo_buffer, sizeof(upload_buffer) - strlen(upload_buffer) - 1);
	strncat(upload_buffer, tail_buffer, sizeof(upload_buffer) - strlen(upload_buffer) - 1);
	memset(vcode->exinfo, 0, sizeof(vcode->exinfo));

	#if IS_ENABLED(CONFIG_FMEA)
	pr_info("Upload data: %s\n", upload_buffer);
	writeData("cds_5130", NULL, upload_buffer);
	#endif
}

static void haptics_vcode_work_routine(struct work_struct *work)
{
	struct vcode_infomation_statistic *vcode = NULL;

	pr_info("Upload vcode data work handler\n");
	vcode = &gchip->vcode;
	mutex_lock(&gchip->lock);
	if (vcode->exception_occur) {
		vcode_haptic_upload_handler();
		vcode->exception_occur = false;
	}
	vcode->delay_work_on = false;
	mutex_unlock(&gchip->lock);
}

int vcode_haptic_statistic(enum haptic_vcode_type exception_type, const char *ex_info)
{
	s64 delta_ms = 0;
	ktime_t new_stamps;
	struct vcode_infomation_statistic *vcode = NULL;

	if (gchip == NULL) {
		pr_err("Error: gchip is NULL\n");
		return -ENOMEM;
	}

	vcode = &gchip->vcode;
	mutex_lock(&gchip->lock);
	// update statistic data
	if (exception_type >= HAPTIC_VCODE_ALL) {
		pr_err("Exception type over range (%d)\n", exception_type);
		goto direct_exit;
	}

	if (vcode->count_strategy[exception_type] > 0) {
		vcode->count_strategy[exception_type]--;
	} else {
		pr_err("Count times zero, %d\n", exception_type);
		goto direct_exit;
	}

	vcode->times_statistic[exception_type]++;
	vcode->exception_occur = true;
	if (ex_info) {
		strncat(vcode->exinfo, ex_info, sizeof(vcode->exinfo) - strlen(vcode->exinfo) - 1);
	}

	new_stamps = ktime_get_real();
	delta_ms = ktime_to_ms(ktime_sub(new_stamps, vcode->upload_time_line));
	if (delta_ms < 10 * 60 * 1000) {
		pr_err("Not upload less 10 minutes, %ld\n", delta_ms);
		if (!vcode->delay_work_on) {
			schedule_delayed_work(&vcode->vcode_work, msecs_to_jiffies(10 * 60 * 1000 - delta_ms));
			vcode->delay_work_on = true;
		}
		goto direct_exit;
	}
	pr_info("Upload duration, %ld\n", delta_ms);
	vcode->upload_time_line = new_stamps;

	vcode_haptic_upload_handler();
	vcode->exception_occur = false;

direct_exit:
	mutex_unlock(&gchip->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(vcode_haptic_statistic);

#endif


/************************************************************************************
 * func:  register a haptic misc device for vibrator
 * @name: haptic misc device name(matching hidl)
 * @hp:   vibrator control interface struct
 *
 ***********************************************************************************/
void haptic_device_set_capcity(struct haptic_handle *hh, int bit)
{
	if (bit > HAPTIC_CNT - 1) {
		pr_err("%s invalid bit, bit=%d\n", __func__, bit);
		return;
	}
	__set_bit(bit, hh->hap_bit);
}
EXPORT_SYMBOL_GPL(haptic_device_set_capcity);


int haptic_handle_create(struct haptic_misc *hap_misc_dev, const char *name)
{
	struct haptic_handle *hh;

	hh = kzalloc(sizeof(struct haptic_handle), GFP_KERNEL);
	if (!hh)
		return -ENOMEM;

	hh->name = name;
	hap_misc_dev->handle = hh;

	// 可以预设一些bit

	return 0;
}
EXPORT_SYMBOL_GPL(haptic_handle_create);


int haptic_miscdev_register(struct haptic_misc *hap_misc_dev)
{
	int ret = 0;

	pr_info("enter, hm=%px, name=%s\n", hap_misc_dev, hap_misc_dev->handle->name);

	mutex_lock(&haptic_lock);

	if (!hap_misc_dev->handle) {
		pr_err("Not initial handle ops\n");
		mutex_unlock(&haptic_lock);
		return -EFAULT;
	}

	hap_misc_dev->name = hap_misc_dev->handle->name;

	hap_misc_dev->misc.fops = &fops;
	hap_misc_dev->misc.minor = MISC_DYNAMIC_MINOR;
	hap_misc_dev->misc.name = hap_misc_dev->handle->name;


	ret = misc_register(&hap_misc_dev->misc);
	if (ret) {
		pr_err("misc register fail, ret=%d\n", ret);
		mutex_unlock(&haptic_lock);
		return ret;
	}

	hap_misc_dev->devt = MKDEV(MISC_MAJOR, hap_misc_dev->misc.minor);

	INIT_LIST_HEAD(&hap_misc_dev->list);
	mutex_init(&hap_misc_dev->lock);

	spin_lock(&haptic_list_lock);
	list_add(&hap_misc_dev->list, &haptic_list);
	spin_unlock(&haptic_list_lock);

	pr_info("end, hap_misc_dev=%px, name=%s\n", hap_misc_dev, hap_misc_dev->name);
	vibrator_count++;
	mutex_unlock(&haptic_lock);
	return 0;

}

EXPORT_SYMBOL_GPL(haptic_miscdev_register);

void haptic_handle_destroy(struct haptic_handle *hh)
{

	if (hh) {
		if (hh->destroy)
			hh->destroy(hh);

		kfree(hh);
		hh = NULL;
	}

}
EXPORT_SYMBOL_GPL(haptic_handle_destroy);


int haptic_miscdev_unregister(struct haptic_misc *hap_misc_dev)
{
	mutex_lock(&haptic_lock);
	if (hap_misc_dev) {

		haptic_handle_destroy(hap_misc_dev->handle);
		misc_deregister(&hap_misc_dev->misc);

		spin_lock(&haptic_list_lock);
		list_del(&hap_misc_dev->list);
		spin_unlock(&haptic_list_lock);

		kfree(hap_misc_dev);
		vibrator_count--;
	}
	mutex_unlock(&haptic_lock);
	return 0;

}

EXPORT_SYMBOL_GPL(haptic_miscdev_unregister);



/*************************************************************
 *
 *               Haptic COre Dev fops
 *
 ***********************************************************/
static int haptic_core_open(struct inode *inode, struct file *file)
{
	pr_info("haptic core open called\n");
	return 0;
}

static ssize_t haptic_core_read(struct file *file, char __user *buff, size_t len, loff_t *offset)
{
	pr_info("haptic core read called\n");
	return 0;
}

static ssize_t haptic_core_write(struct file *file, const char __user *buff, size_t len, loff_t *off)
{
	pr_info("haptic core write called, len:%d\n", len);
	return len;
}

static int haptic_core_release(struct inode *inode, struct file *file)
{
	pr_info("haptic core release called\n");
	//file->private_data = (void *)NULL;
	return 0;
}

static long haptic_core_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int __user *vib_count = (int __user *)arg;


	switch (cmd) {

	case HAPTIC_CORE_GET_VIB_COUNT:
		if (copy_to_user(vib_count, &vibrator_count, sizeof(int))) {
			pr_err("Haptic core get vib count failed\n");
			return -EINVAL;
		}
		break;

	default:
		pr_info("haptic core ioctl invalid cmd:, %#x\n", cmd);
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long haptic_core_ioctl_compat(struct file *file, unsigned int cmd, unsigned long arg)
{
	pr_info("32bit compat mode, to be deal\n");

	return -EPERM;
}
#endif

static struct file_operations haptic_core_fops = {
	.owner = THIS_MODULE,
	.open = haptic_core_open,
	.read = haptic_core_read,
	.write = haptic_core_write,
	.unlocked_ioctl = haptic_core_ioctl,

#ifdef CONFIG_COMPAT
	.compat_ioctl = haptic_core_ioctl_compat,
#endif

	.release = haptic_core_release,
};

static int __init haptic_miscdev_init(void)
{
	int ret = 0;
	struct haptic_core_dev *chip = NULL;

	haptic_core_misc.fops = &haptic_core_fops;
	haptic_core_misc.minor = MISC_DYNAMIC_MINOR;
	haptic_core_misc.name = "vivo_haptic_core";

	ret = misc_register(&haptic_core_misc);
	if (ret) {
		pr_err("haptic core misc register fail, ret=%d\n", ret);
		//return ret;
	}

	chip = kzalloc(sizeof(struct haptic_core_dev), GFP_KERNEL);
	if (!chip) {
		pr_err("Haptic core mem alloc faile\n");
		gchip = NULL;
	} else {
		mutex_init(&chip->lock);
		#ifdef VIVO_HAPTIC_VCODE
		INIT_DELAYED_WORK(&chip->vcode.vcode_work, haptics_vcode_work_routine);
		vcode_haptic_init(&chip->vcode);
		#endif
		gchip = chip;
	}

	return 0;
}

device_initcall(haptic_miscdev_init);
MODULE_AUTHOR("zhang xiaodong <xiaodong.zhang@vivo.com>");
MODULE_LICENSE("GPL");
