// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018 Samsung Electronics Co., Ltd.
 */


#include "ufsfeature.h"
#include "ufshcd.h"
#include "ufs_quirks.h"

#if defined(CONFIG_SCSI_UFS_HPB)
#include "ufshpb.h"
#endif

#define QUERY_REQ_TIMEOUT				1500 /* msec */

static inline void ufsf_init_query(struct ufs_hba *hba,
				   struct ufs_query_req **request,
				   struct ufs_query_res **response,
				   enum query_opcode opcode, u8 idn,
				   u8 index, u8 selector)
{
	*request = &hba->dev_cmd.query.request;
	*response = &hba->dev_cmd.query.response;
	memset(*request, 0, sizeof(struct ufs_query_req));
	memset(*response, 0, sizeof(struct ufs_query_res));
	(*request)->upiu_req.opcode = opcode;
	(*request)->upiu_req.idn = idn;
	(*request)->upiu_req.index = index;
	(*request)->upiu_req.selector = selector;
}

/*
 * ufs feature common functions.
 */
int ufsf_query_flag(struct ufs_hba *hba, enum query_opcode opcode,
		    enum flag_idn idn, u8 index, bool *flag_res)
{
	struct ufs_query_req *request = NULL;
	struct ufs_query_res *response = NULL;
	u8 selector;
	int err;

	BUG_ON(!hba);

	ufshcd_hold(hba, false);
	mutex_lock(&hba->dev_cmd.lock);

	if (hba->dev_info.wmanufacturerid == UFS_VENDOR_SAMSUNG ||
		hba->dev_info.wmanufacturerid == UFS_VENDOR_MICRON)
		selector = UFSFEATURE_SELECTOR;
	else
		selector = 0;

	/*
	 * Init the query response and request parameters
	 */
	ufsf_init_query(hba, &request, &response, opcode, idn, index,
			selector);

	switch (opcode) {
	case UPIU_QUERY_OPCODE_SET_FLAG:
	case UPIU_QUERY_OPCODE_CLEAR_FLAG:
	case UPIU_QUERY_OPCODE_TOGGLE_FLAG:
		request->query_func = UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST;
		break;
	case UPIU_QUERY_OPCODE_READ_FLAG:
		request->query_func = UPIU_QUERY_FUNC_STANDARD_READ_REQUEST;
		if (!flag_res) {
			/* No dummy reads */
			dev_err(hba->dev, "%s: Invalid argument for read request\n",
					__func__);
			err = -EINVAL;
			goto out_unlock;
		}
		break;
	default:
		dev_err(hba->dev,
			"%s: Expected query flag opcode but got = %d\n",
			__func__, opcode);
		err = -EINVAL;
		goto out_unlock;
	}

	/* Send query request */
	err = ufshcd_exec_dev_cmd(hba, DEV_CMD_TYPE_QUERY, QUERY_REQ_TIMEOUT);
	if (err) {
		dev_err(hba->dev,
			"%s: Sending flag query for idn %d failed, err = %d\n",
			__func__, idn, err);
		goto out_unlock;
	}

	if (flag_res)
		*flag_res = (be32_to_cpu(response->upiu_res.value) &
				MASK_QUERY_UPIU_FLAG_LOC) & 0x1;

out_unlock:
	mutex_unlock(&hba->dev_cmd.lock);
	ufshcd_release(hba);
	return err;
}

int ufsf_query_flag_retry(struct ufs_hba *hba, enum query_opcode opcode,
			  enum flag_idn idn, u8 idx, bool *flag_res)
{
	int ret;
	int retries;

	for (retries = 0; retries < UFSF_QUERY_REQ_RETRIES; retries++) {
		ret = ufsf_query_flag(hba, opcode, idn, idx, flag_res);
		if (ret)
			dev_dbg(hba->dev,
				"%s: failed with error %d, retries %d\n",
				__func__, ret, retries);
		else
			break;
	}
	if (ret)
		dev_err(hba->dev,
			"%s: query flag, opcode %d, idn %d, failed with error %d after %d retires\n",
			__func__, opcode, idn, ret, retries);
	return ret;
}

int ufsf_query_attr_retry(struct ufs_hba *hba, enum query_opcode opcode,
			  enum attr_idn idn, u8 idx, u32 *attr_val)
{
	int ret;
	int retries;
	u8 selector;

	if (hba->dev_info.wmanufacturerid == UFS_VENDOR_SAMSUNG ||
		hba->dev_info.wmanufacturerid == UFS_VENDOR_MICRON)
		selector = UFSFEATURE_SELECTOR;
	else
		selector = 0;

	for (retries = 0; retries < UFSF_QUERY_REQ_RETRIES; retries++) {
		ret = ufshcd_query_attr(hba, opcode, idn, idx,
					selector, attr_val);
		if (ret)
			dev_dbg(hba->dev,
				"%s: failed with error %d, retries %d\n",
				__func__, ret, retries);
		else
			break;
	}
	if (ret)
		dev_err(hba->dev,
			"%s: query attr, opcode %d, idn %d, failed with error %d after %d retires\n",
			__func__, opcode, idn, ret, retries);
	return ret;
}

static int ufsf_read_desc(struct ufs_hba *hba, u8 desc_id, u8 desc_index,
			  u8 selector, u8 *desc_buf, u32 size)
{
	int err = 0;
	bool pm_resumed = false;

	if (!hba->pm_op_in_progress) {
		pm_runtime_get_sync(hba->dev);
		pm_resumed = true;
	}

	err = ufshcd_query_descriptor_retry(hba, UPIU_QUERY_OPCODE_READ_DESC,
					    desc_id, desc_index,
					    selector,
					    desc_buf, &size);
	if (err)
		ERR_MSG("reading Device Desc failed. err = %d", err);
	if (pm_resumed)
		pm_runtime_put_sync(hba->dev);

	return err;
}

static int ufsf_read_dev_desc(struct ufsf_feature *ufsf, u8 selector)
{
	u8 desc_buf[UFSF_QUERY_DESC_DEVICE_MAX_SIZE] = {0};
	int ret;

	ret = ufsf_read_desc(ufsf->hba, QUERY_DESC_IDN_DEVICE, 0, selector,
			     desc_buf, UFSF_QUERY_DESC_DEVICE_MAX_SIZE);
	if (ret)
		return ret;

	ufsf->num_lu = desc_buf[DEVICE_DESC_PARAM_NUM_LU];
	INIT_INFO("device lu count %d", ufsf->num_lu);

	INIT_INFO("sel=%u length=%u(0x%x) bSupport=0x%.2x, extend=0x%.2x_%.2x",
		  selector, desc_buf[DEVICE_DESC_PARAM_LEN],
		  desc_buf[DEVICE_DESC_PARAM_LEN],
		  desc_buf[DEVICE_DESC_PARAM_FEAT_SUP],
		  desc_buf[DEVICE_DESC_PARAM_EX_FEAT_SUP+2],
		  desc_buf[DEVICE_DESC_PARAM_EX_FEAT_SUP+3]);
	INIT_INFO("Driver Feature Version : (%.6X%s)", UFSFEATURE_DD_VER,
		UFSFEATURE_DD_VER_POST);
#if defined(CONFIG_SCSI_UFS_HPB)
	ufshpb_get_dev_info(&ufsf->hpb_dev_info, desc_buf);
#endif

#if defined(CONFIG_SCSI_UFS_TW)
	ufstw_get_dev_info(&ufsf->tw_dev_info, desc_buf);
#endif

#if IS_ENABLED(CONFIG_UFSHID)
	if (ufsf->hba->dev_info.wmanufacturerid == UFS_VENDOR_SAMSUNG)
		ufshid_get_dev_info(ufsf, desc_buf);
#endif
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	ufsringbuf_get_dev_info(ufsf, desc_buf);
#endif

	return 0;
}

static int ufsf_read_geo_desc(struct ufsf_feature *ufsf, u8 selector)
{
	u8 geo_buf[UFSF_QUERY_DESC_GEOMETRY_MAX_SIZE];
	int ret;

	ret = ufsf_read_desc(ufsf->hba, QUERY_DESC_IDN_GEOMETRY, 0, selector,
			     geo_buf, UFSF_QUERY_DESC_GEOMETRY_MAX_SIZE);
	if (ret)
		return ret;

#if defined(CONFIG_SCSI_UFS_HPB)
	if (ufsf->hpb_dev_info.hpb_device)
		ufshpb_get_geo_info(&ufsf->hpb_dev_info, geo_buf);
#endif

#if defined(CONFIG_SCSI_UFS_TW)
	if (ufsf->tw_dev_info.tw_device)
		ufstw_get_geo_info(&ufsf->tw_dev_info, geo_buf);
#endif

#if IS_ENABLED(CONFIG_UFSRINGBUF)
	if (ufsringbuf_get_state(ufsf) == RINGBUF_NEED_INIT)
		ufsringbuf_get_geo_info(ufsf, geo_buf);
#endif

	return 0;
}

static int ufsf_read_unit_desc(struct ufsf_feature *ufsf,
			       unsigned int lun, u8 selector)
{
	u8 unit_buf[UFSF_QUERY_DESC_UNIT_MAX_SIZE];
	int lu_enable, ret = 0;

	ret = ufsf_read_desc(ufsf->hba, QUERY_DESC_IDN_UNIT, lun, selector,
			     unit_buf, UFSF_QUERY_DESC_UNIT_MAX_SIZE);
	if (ret) {
		ERR_MSG("read unit desc failed. ret %d", ret);
		goto out;
	}

	lu_enable = unit_buf[UNIT_DESC_PARAM_LU_ENABLE];
	if (!lu_enable)
		return 0;

#if defined(CONFIG_SCSI_UFS_HPB)
	if (ufsf->hpb_dev_info.hpb_device) {
		ret = ufshpb_get_lu_info(ufsf, lun, unit_buf);
		if (ret == -ENOMEM)
			goto out;
	}
#endif

#if defined(CONFIG_SCSI_UFS_TW)
	if (ufsf->tw_dev_info.tw_device) {
		ret = ufstw_get_lu_info(ufsf, lun, unit_buf);
		if (ret == -ENOMEM)
			goto out;
	}
#endif
out:
	return ret;
}

void ufsf_device_check(struct ufs_hba *hba)
{
	struct ufsf_feature *ufsf = &hba->ufsf;
	int ret;
	unsigned int lun;
	u8 selector = 0;

	atomic_set(&ufsf->slave_conf_cnt, 0);

	ufsf->hba = hba;

	if (hba->dev_info.wmanufacturerid == UFS_VENDOR_SAMSUNG ||
		hba->dev_info.wmanufacturerid == UFS_VENDOR_MICRON)
		selector = UFSFEATURE_SELECTOR;

	ret = ufsf_read_dev_desc(ufsf, selector);
	if (ret)
		return;

	ret = ufsf_read_geo_desc(ufsf, selector);
	if (ret)
		return;

	seq_scan_lu(lun) {
		ret = ufsf_read_unit_desc(ufsf, lun, selector);
		if (ret == -ENOMEM)
			goto out_free_mem;
	}

	return;
out_free_mem:
#if defined(CONFIG_SCSI_UFS_HPB)
	seq_scan_lu(lun)
		kfree(ufsf->ufshpb_lup[lun]);

	/* don't call init handler */
	ufsf->ufshpb_state = HPB_NOT_SUPPORTED;
#endif
#if defined(CONFIG_SCSI_UFS_TW)
	seq_scan_lu(lun)
		kfree(ufsf->tw_lup[lun]);

	ufsf->tw_dev_info.tw_device = false;
	atomic_set(&ufsf->tw_state, TW_NOT_SUPPORTED);
#endif
	return;
}

static void ufsf_print_query_buf(unsigned char *field, int size)
{
	unsigned char buf[255];
	unsigned int count = 0;
	int i;

	count += snprintf(buf, 8, "(0x00):");

	for (i = 0; i < size; i++) {
		count += snprintf(buf + count, 4, " %.2X", field[i]);

		if ((i + 1) % 16 == 0) {
			buf[count] = '\n';
			buf[count + 1] = '\0';
			printk(buf);
			count = 0;
			count += snprintf(buf, 8, "(0x%.2X):", i + 1);
		} else if ((i + 1) % 4 == 0)
			count += snprintf(buf + count, 3, " :");
	}
	buf[count] = '\n';
	buf[count + 1] = '\0';
	printk(buf);
}

inline int ufsf_check_query(__u32 opcode)
{
	return (opcode & 0xffff0000) >> 16 == UFSFEATURE_QUERY_OPCODE;
}

int ufsf_query_ioctl(struct ufsf_feature *ufsf, unsigned int lun,
		     void __user *buffer,
		     struct ufs_ioctl_query_data_hpb *ioctl_data, u8 selector)
{
	unsigned char *kernel_buf;
	int opcode;
	int err = 0;
	int index = 0;
	int length = 0;
	int buf_len = 0;

	opcode = ioctl_data->opcode & 0xffff;

	INFO_MSG("op %u idn %u sel %u size %u(0x%X)", opcode, ioctl_data->idn,
		 selector, ioctl_data->buf_size, ioctl_data->buf_size);

	buf_len = (ioctl_data->idn == QUERY_DESC_IDN_STRING) ?
		IOCTL_DEV_CTX_MAX_SIZE : QUERY_DESC_MAX_SIZE;
	if (ioctl_data->buf_size > buf_len) {
		err = -EINVAL;
		goto out;
	}

	kernel_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!kernel_buf) {
		err = -ENOMEM;
		goto out;
	}

	switch (opcode) {
	case UPIU_QUERY_OPCODE_WRITE_DESC:
		err = copy_from_user(kernel_buf, buffer +
				     sizeof(struct ufs_ioctl_query_data_hpb),
				     ioctl_data->buf_size);
		INFO_MSG("buf size %d", ioctl_data->buf_size);
		ufsf_print_query_buf(kernel_buf, ioctl_data->buf_size);
		if (err)
			goto out_release_mem;
		break;

	case UPIU_QUERY_OPCODE_READ_DESC:
		switch (ioctl_data->idn) {
		case QUERY_DESC_IDN_UNIT:
			if (!ufs_is_valid_unit_desc_lun(lun)) {
				ERR_MSG("No unit descriptor for lun 0x%x", lun);
				err = -EINVAL;
				goto out_release_mem;
			}
			index = lun;
			INFO_MSG("read lu desc lun: %d", index);
			break;

		case QUERY_DESC_IDN_STRING:
#if defined(CONFIG_SCSI_UFS_HPB)
			if (!ufs_is_valid_unit_desc_lun(lun)) {
				ERR_MSG("No unit descriptor for lun 0x%x", lun);
				err = -EINVAL;
				goto out_release_mem;
			}
			err = ufshpb_issue_req_dev_ctx(ufsf->ufshpb_lup[lun],
						       kernel_buf,
						       ioctl_data->buf_size);
			if (err < 0)
				goto out_release_mem;

			goto copy_buffer;
#endif
		case QUERY_DESC_IDN_DEVICE:
		case QUERY_DESC_IDN_GEOMETRY:
		case QUERY_DESC_IDN_CONFIGURATION:
			break;

		default:
			ERR_MSG("invalid idn %d", ioctl_data->idn);
			err = -EINVAL;
			goto out_release_mem;
		}
		break;
	default:
		ERR_MSG("invalid opcode %d", opcode);
		err = -EINVAL;
		goto out_release_mem;
	}

	length = ioctl_data->buf_size;

	err = ufshcd_query_descriptor_retry(ufsf->hba, opcode, ioctl_data->idn,
					    index, selector, kernel_buf,
					    &length);
	if (err)
		goto out_release_mem;

#if defined(CONFIG_SCSI_UFS_HPB)
copy_buffer:
#endif
	if (opcode == UPIU_QUERY_OPCODE_READ_DESC) {
		err = copy_to_user(buffer, ioctl_data,
				   sizeof(struct ufs_ioctl_query_data_hpb));
		if (err)
			ERR_MSG("Failed copying back to user.");

		err = copy_to_user(buffer + sizeof(struct ufs_ioctl_query_data_hpb),
				   kernel_buf, ioctl_data->buf_size);
		if (err)
			ERR_MSG("Fail: copy rsp_buffer to user space.");
	}
out_release_mem:
	kfree(kernel_buf);
out:
	return err;
}

static inline void ufsf_set_read10_debug_cmd(unsigned char *cdb, int lba,
					     int len)
{
	cdb[0] = READ_10;
	cdb[1] = 0x02;
	cdb[2] = GET_BYTE_3(lba);
	cdb[3] = GET_BYTE_2(lba);
	cdb[4] = GET_BYTE_1(lba);
	cdb[5] = GET_BYTE_0(lba);
	cdb[6] = GET_BYTE_2(len);
	cdb[7] = GET_BYTE_1(len);
	cdb[8] = GET_BYTE_0(len);
}

bool ufsf_upiu_check_for_ccd(struct ufshcd_lrb *lrbp)
{
	unsigned char *cdb = lrbp->cmd->cmnd;
	int data_seg_len, sense_data_len;

	if (cdb[0] != VENDOR_OP || cdb[1] != VENDOR_CCD)
		return false;

	data_seg_len = be32_to_cpu(lrbp->ucd_rsp_ptr->header.dword_2) &
					MASK_RSP_UPIU_DATA_SEG_LEN;
	sense_data_len = be16_to_cpu(lrbp->ucd_rsp_ptr->sr.sense_data_len);

	if (data_seg_len != CCD_DATA_SEG_LEN ||
		sense_data_len != CCD_SENSE_DATA_LEN) {
		WARN_MSG("CCD info is wrong. so check it.");
		WARN_MSG("CCD data_seg_len = %d, sense_data_len %d",
			data_seg_len, sense_data_len);
	} else {
		INFO_MSG("CCD info is correct!!");
	}

	/*
	 * sense_len will be not set as Descriptor Type isn't 0x70
	 * if not set sense_len, sense will not be able to copy
	 * in sg_scsi_ioctl()
	 */
	scsi_req(lrbp->cmd->request)->sense_len = CCD_SENSE_DATA_LEN;

	return true;
}

inline int ufsf_get_scsi_device(struct ufs_hba *hba, struct scsi_device *sdev)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(hba->host->host_lock, flags);
	ret = scsi_device_get(sdev);
	if (!ret && !scsi_device_online(sdev)) {
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		scsi_device_put(sdev);
		ERR_MSG("scsi_device_get failed.(%d)", ret);
		return -ENODEV;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return ret;
}

inline bool ufsf_is_valid_lun(int lun)
{
	return lun < UFS_UPIU_MAX_GENERAL_LUN;
}

inline void ufsf_slave_configure(struct ufsf_feature *ufsf,
				 struct scsi_device *sdev)
{
	if (ufsf_is_valid_lun(sdev->lun)) {
		ufsf->sdev_ufs_lu[sdev->lun] = sdev;
		atomic_inc(&ufsf->slave_conf_cnt);
		INFO_MSG("lun[%d] sdev(%p) q(%p) slave_conf_cnt(%d/%d)",
			 (int)sdev->lun, sdev, sdev->request_queue,
			 atomic_read(&ufsf->slave_conf_cnt), ufsf->num_lu);
	}
}

inline void ufsf_prep_fn(struct ufsf_feature *ufsf,
			 struct ufshcd_lrb *lrbp)
{
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	if (ufsringbuf_get_state(ufsf) == RINGBUF_PRESENT ||
	    ufsringbuf_get_state(ufsf) == RINGBUF_RESET)
		ufsringbuf_prep_fn(ufsf, lrbp);
#endif
}

inline void ufsf_reset_host(struct ufsf_feature *ufsf)
{
#if IS_ENABLED(CONFIG_UFSHID)
	INFO_MSG("run reset_host.. hid_state(%d) -> HID_RESET",
		 ufshid_get_state(ufsf));
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_reset_host(ufsf);
#endif
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	INFO_MSG("run reset_host.. ringbuf_state(%d) -> RINGBUF_RESET",
		 ufsringbuf_get_state(ufsf));
	if (ufsringbuf_get_state(ufsf) == RINGBUF_PRESENT)
		ufsringbuf_reset_host(ufsf);
#endif
}

inline void ufsf_init(struct ufsf_feature *ufsf)
{
#if IS_ENABLED(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_NEED_INIT)
		ufshid_init(ufsf);
#endif
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	if (ufsringbuf_get_state(ufsf) == RINGBUF_NEED_INIT)
		ufsringbuf_init(ufsf);
#endif
}

inline void ufsf_reset(struct ufsf_feature *ufsf)
{
#if IS_ENABLED(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_RESET)
		ufshid_reset(ufsf);
#endif
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	if (ufsringbuf_get_state(ufsf) == RINGBUF_RESET)
		ufsringbuf_reset(ufsf);
#endif
}

inline void ufsf_remove(struct ufsf_feature *ufsf)
{
#if IS_ENABLED(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_remove(ufsf);
#endif
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	if (ufsringbuf_get_state(ufsf) == RINGBUF_PRESENT)
		ufsringbuf_remove(ufsf);
#endif
}

inline void ufsf_set_init_state(struct ufsf_feature *ufsf)
{
	atomic_set(&ufsf->slave_conf_cnt, 0);
	ufsf->issue_ioctl = false;
#if IS_ENABLED(CONFIG_UFSHID)
	ufshid_set_state(ufsf, HID_NEED_INIT);
#endif
#if IS_ENABLED(CONFIG_UFSRINGBUF)
	ufsringbuf_set_state(ufsf, RINGBUF_NEED_INIT);
#endif
}

inline void ufsf_suspend(struct ufsf_feature *ufsf)
{
#if IS_ENABLED(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_suspend(ufsf);
#endif
}

inline void ufsf_resume(struct ufsf_feature *ufsf)
{
#if IS_ENABLED(CONFIG_UFSHID)
		if (ufshid_get_state(ufsf) == HID_SUSPEND)
			ufshid_resume(ufsf);
#endif

#if IS_ENABLED(CONFIG_UFSRINGBUF)
	if (ufsringbuf_get_state(ufsf) == RINGBUF_PRESENT)
		ufsringbuf_resume(ufsf);
#endif
}

inline void ufsf_on_idle(struct ufsf_feature *ufsf, bool scsi_req)
{
#if IS_ENABLED(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_PRESENT &&
	    !ufsf->hba->outstanding_reqs && scsi_req)
		ufshid_on_idle(ufsf);
#endif
}

inline void ufsf_change_lun(struct ufsf_feature *ufsf,
			    struct ufshcd_lrb *lrbp)
{
	int ctx_lba = LI_EN_32(lrbp->cmd->cmnd + 2);

	if (unlikely(ufsf->issue_ioctl == true &&
	    ctx_lba == READ10_DEBUG_LBA)) {
		lrbp->lun = READ10_DEBUG_LUN;
		INFO_MSG("lun 0x%X lba 0x%X", lrbp->lun, ctx_lba);
	}
}

inline int ufsf_get_ee_status(struct ufs_hba *hba, u32 *status)
{
	return ufsf_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,
				     QUERY_ATTR_IDN_EE_STATUS, 0, status);
}

/*
 * Wrapper functions for ufshpb.
 */
#if defined(CONFIG_SCSI_UFS_HPB)
inline int ufsf_hpb_prepare_pre_req(struct ufsf_feature *ufsf,
				    struct scsi_cmnd *cmd, int lun)
{
	if (ufsf->ufshpb_state == HPB_PRESENT)
		return ufshpb_prepare_pre_req(ufsf, cmd, lun);
	return -ENODEV;
}

inline int ufsf_hpb_prepare_add_lrbp(struct ufsf_feature *ufsf, int add_tag)
{
	if (ufsf->ufshpb_state == HPB_PRESENT)
		return ufshpb_prepare_add_lrbp(ufsf, add_tag);
	return -ENODEV;
}

inline void ufsf_hpb_end_pre_req(struct ufsf_feature *ufsf,
				 struct request *req)
{
	ufshpb_end_pre_req(ufsf, req);
}

inline void ufsf_hpb_change_lun(struct ufsf_feature *ufsf,
				struct ufshcd_lrb *lrbp)
{
	int ctx_lba = LI_EN_32(lrbp->cmd->cmnd + 2);

	if (ufsf->ufshpb_state == HPB_PRESENT &&
	    ufsf->issue_ioctl == true && ctx_lba == READ10_DEBUG_LBA) {
		lrbp->lun = READ10_DEBUG_LUN;
		INFO_MSG("lun 0x%X lba 0x%X", lrbp->lun, ctx_lba);
	}
}

inline void ufsf_hpb_prep_fn(struct ufsf_feature *ufsf,
			     struct ufshcd_lrb *lrbp)
{
	if (ufsf->ufshpb_state == HPB_PRESENT
	    && ufsf->issue_ioctl == false)
		ufshpb_prep_fn(ufsf, lrbp);
}

inline void ufsf_hpb_noti_rb(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp)
{
	if (ufsf->ufshpb_state == HPB_PRESENT)
		ufshpb_rsp_upiu(ufsf, lrbp);
}

inline void ufsf_hpb_reset_lu(struct ufsf_feature *ufsf)
{
	ufsf->ufshpb_state = HPB_RESET;
	schedule_work(&ufsf->ufshpb_reset_work);
}

inline void ufsf_hpb_reset_host(struct ufsf_feature *ufsf)
{
	if (ufsf->ufshpb_state == HPB_PRESENT)
		ufsf->ufshpb_state = HPB_RESET;
}

inline void ufsf_hpb_init(struct ufsf_feature *ufsf)
{
	if (ufsf->hpb_dev_info.hpb_device &&
	    ufsf->ufshpb_state == HPB_NEED_INIT) {
		INIT_WORK(&ufsf->ufshpb_init_work, ufshpb_init_handler);
		schedule_work(&ufsf->ufshpb_init_work);
	}
}

inline void ufsf_hpb_reset(struct ufsf_feature *ufsf)
{
	if (ufsf->hpb_dev_info.hpb_device &&
	    ufsf->ufshpb_state == HPB_RESET)
		schedule_work(&ufsf->ufshpb_reset_work);
}

inline void ufsf_hpb_suspend(struct ufsf_feature *ufsf)
{
	if (ufsf->ufshpb_state == HPB_PRESENT)
		ufshpb_suspend(ufsf);
}

inline void ufsf_hpb_resume(struct ufsf_feature *ufsf)
{
	if (ufsf->ufshpb_state == HPB_PRESENT)
		ufshpb_resume(ufsf);
}

inline void ufsf_hpb_release(struct ufsf_feature *ufsf)
{
	ufshpb_release(ufsf, HPB_NEED_INIT);
}

inline void ufsf_hpb_set_init_state(struct ufsf_feature *ufsf)
{
	ufsf->ufshpb_state = HPB_NEED_INIT;
}
#else
inline int ufsf_hpb_prepare_pre_req(struct ufsf_feature *ufsf,
				    struct scsi_cmnd *cmd, int lun)
{
	return 0;
}

inline int ufsf_hpb_prepare_add_lrbp(struct ufsf_feature *ufsf, int add_tag)
{
	return 0;
}

inline void ufsf_hpb_end_pre_req(struct ufsf_feature *ufsf,
				 struct request *req) {}
inline void ufsf_hpb_change_lun(struct ufsf_feature *ufsf,
				struct ufshcd_lrb *lrbp) {}
inline void ufsf_hpb_prep_fn(struct ufsf_feature *ufsf,
			     struct ufshcd_lrb *lrbp) {}
inline void ufsf_hpb_noti_rb(struct ufsf_feature *ufsf,
			     struct ufshcd_lrb *lrbp) {}
inline void ufsf_hpb_reset_lu(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_reset_host(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_init(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_reset(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_suspend(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_resume(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_release(struct ufsf_feature *ufsf) {}
inline void ufsf_hpb_set_init_state(struct ufsf_feature *ufsf) {}
#endif

/*
 * Wrapper functions for ufstw.
 */

#if defined(CONFIG_SCSI_UFS_TW)
inline void ufsf_tw_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp)
{
	ufstw_prep_fn(ufsf, lrbp);
}

inline void ufsf_tw_init(struct ufsf_feature *ufsf)
{
	INIT_INFO("init start.. tw_state %d\n",
		  atomic_read(&ufsf->tw_state));

	if (ufsf->tw_dev_info.tw_device &&
	    atomic_read(&ufsf->tw_state) == TW_NEED_INIT) {
		INIT_WORK(&ufsf->tw_init_work, ufstw_init_work_fn);
		schedule_work(&ufsf->tw_init_work);
	}
}

inline void ufsf_tw_reset(struct ufsf_feature *ufsf)
{
	INIT_INFO("reset start.. tw_state %d\n",
		  atomic_read(&ufsf->tw_state));

	if (ufsf->tw_dev_info.tw_device &&
	    atomic_read(&ufsf->tw_state) == TW_RESET)
		schedule_work(&ufsf->tw_reset_work);
}

inline void ufsf_tw_suspend(struct ufsf_feature *ufsf)
{
	if (atomic_read(&ufsf->tw_state) == TW_PRESENT)
		ufstw_suspend(ufsf);
}

inline void ufsf_tw_resume(struct ufsf_feature *ufsf)
{
	if (atomic_read(&ufsf->tw_state) == TW_PRESENT)
		ufstw_resume(ufsf);
}

inline void ufsf_tw_release(struct ufsf_feature *ufsf)
{
	ufstw_release(&ufsf->tw_kref);
}

inline void ufsf_tw_set_init_state(struct ufsf_feature *ufsf)
{
	atomic_set(&ufsf->tw_state, TW_NEED_INIT);
}

inline void ufsf_tw_reset_lu(struct ufsf_feature *ufsf)
{
	INFO_MSG("run reset_lu.. tw_state(%d) -> TW_RESET",
		 atomic_read(&ufsf->tw_state));
	atomic_set(&ufsf->tw_state, TW_RESET);
	if (ufsf->tw_dev_info.tw_device)
		schedule_work(&ufsf->tw_reset_work);
}

inline void ufsf_tw_reset_host(struct ufsf_feature *ufsf)
{
	INFO_MSG("run reset_host.. tw_state(%d) -> TW_RESET",
		 atomic_read(&ufsf->tw_state));
	if (atomic_read(&ufsf->tw_state) == TW_PRESENT)
		atomic_set(&ufsf->tw_state, TW_RESET);
}

inline void ufsf_tw_ee_handler(struct ufsf_feature *ufsf)
{
	u32 status = 0;
	int err;

	if (ufsf->tw_debug && (atomic_read(&ufsf->tw_state) != TW_PRESENT)) {
		ERR_MSG("tw_state %d", atomic_read(&ufsf->tw_state));
		return;
	}

	if ((atomic_read(&ufsf->tw_state) == TW_PRESENT)
	    && (ufsf->tw_ee_mode == TW_EE_MODE_AUTO)) {
		err = ufsf_get_ee_status(ufsf->hba, &status);
		if (err) {
			dev_err(ufsf->hba->dev,
				"%s: failed to get tw ee status %d\n",
				__func__, err);
			return;
		}
		if (status & MASK_EE_TW)
			ufstw_ee_handler(ufsf);
	}
}
#else
inline void ufsf_tw_prep_fn(struct ufsf_feature *ufsf,
			    struct ufshcd_lrb *lrbp) {}
inline void ufsf_tw_init(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_reset(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_suspend(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_resume(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_release(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_set_init_state(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_reset_lu(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_reset_host(struct ufsf_feature *ufsf) {}
inline void ufsf_tw_ee_handler(struct ufsf_feature *ufsf) {}
#endif
