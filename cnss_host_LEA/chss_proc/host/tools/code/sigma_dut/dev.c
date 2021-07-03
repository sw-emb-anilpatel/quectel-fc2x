/*
 * Sigma Control API DUT (station/AP/sniffer)
 * Copyright (c) 2011-2013, 2017, Qualcomm Atheros, Inc.
 * Copyright (c) 2018-2019, The Linux Foundation
 * All Rights Reserved.
 * Licensed under the Clear BSD license. See README for more details.
 */

#include "sigma_dut.h"
#include "miracast.h"
#include <sys/wait.h>
#include "wpa_ctrl.h"
#include "wpa_helpers.h"


static enum sigma_cmd_result cmd_dev_send_frame(struct sigma_dut *dut,
						struct sigma_conn *conn,
						struct sigma_cmd *cmd)
{
#ifdef MIRACAST
	const char *program = get_param(cmd, "Program");

	if (program && (strcasecmp(program, "WFD") == 0 ||
			strcasecmp(program, "DisplayR2") == 0))
		return miracast_dev_send_frame(dut, conn, cmd);
#endif /* MIRACAST */

	if (dut->mode == SIGMA_MODE_STATION ||
	    dut->mode == SIGMA_MODE_UNKNOWN) {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "Convert "
				"dev_send_frame to sta_send_frame");
		return cmd_sta_send_frame(dut, conn, cmd);
	}

	if (dut->mode == SIGMA_MODE_AP) {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "Convert "
				"dev_send_frame to ap_send_frame");
		return cmd_ap_send_frame(dut, conn, cmd);
	}

#ifdef CONFIG_WLANTEST
	sigma_dut_print(dut, DUT_MSG_DEBUG, "Convert dev_send_frame to "
			"wlantest_send_frame");
	return cmd_wlantest_send_frame(dut, conn, cmd);
#else /* CONFIG_WLANTEST */
	send_resp(dut, conn, SIGMA_ERROR,
		  "errorCode,Unsupported dev_send_frame");
	return STATUS_SENT;
#endif /* CONFIG_WLANTEST */
}


static enum sigma_cmd_result cmd_dev_set_parameter(struct sigma_dut *dut,
						   struct sigma_conn *conn,
						   struct sigma_cmd *cmd)
{
	const char *device = get_param(cmd, "Device");

	if (device && strcasecmp(device, "STA") == 0) {
		sigma_dut_print(dut, DUT_MSG_DEBUG, "Convert "
				"dev_set_parameter to sta_set_parameter");
		return cmd_sta_set_parameter(dut, conn, cmd);
	}

	return INVALID_SEND_STATUS;
}


static enum sigma_cmd_result cmd_dev_exec_action(struct sigma_dut *dut,
						 struct sigma_conn *conn,
						 struct sigma_cmd *cmd)
{
	const char *program = get_param(cmd, "Program");

#ifdef MIRACAST
	if (program && (strcasecmp(program, "WFD") == 0 ||
			strcasecmp(program, "DisplayR2") == 0)) {
		if (get_param(cmd, "interface") == NULL)
			return INVALID_SEND_STATUS;
		return miracast_dev_exec_action(dut, conn, cmd);
	}
#endif /* MIRACAST */

	if (program && strcasecmp(program, "DPP") == 0)
		return dpp_dev_exec_action(dut, conn, cmd);

	return ERROR_SEND_STATUS;
}


static enum sigma_cmd_result cmd_dev_configure_ie(struct sigma_dut *dut,
						  struct sigma_conn *conn,
						  struct sigma_cmd *cmd)
{
	const char *ie_name = get_param(cmd, "IE_Name");
	const char *contents = get_param(cmd, "Contents");

	if (!ie_name || !contents)
		return INVALID_SEND_STATUS;

	if (strcasecmp(ie_name, "RSNE") != 0) {
		send_resp(dut, conn, SIGMA_ERROR,
			  "errorCode,Unsupported IE_Name value");
		return STATUS_SENT;
	}

	free(dut->rsne_override);
	dut->rsne_override = strdup(contents);

	return dut->rsne_override ? SUCCESS_SEND_STATUS : ERROR_SEND_STATUS;
}


static enum sigma_cmd_result cmd_dev_ble_action(struct sigma_dut *dut,
						struct sigma_conn *conn,
						struct sigma_cmd *cmd)
{
#ifdef ANDROID
	char buf[200];
	const char *ble_op = get_param(cmd, "BLEOp");
	const char *prog = get_param(cmd, "Prog");
	const char *service_name = get_param(cmd, "ServiceName");
	const char *ble_role = get_param(cmd, "BLERole");
	const char *discovery_type = get_param(cmd, "DiscoveryType");
	const char *msg_type = get_param(cmd, "messagetype");
	const char *action = get_param(cmd, "action");
	const char *M2Transmit = get_param(cmd, "M2Transmit");
	char *argv[17];
	pid_t pid;

	if (prog && ble_role && action && msg_type) {
		send_resp(dut, conn, SIGMA_COMPLETE,
			  "OrgID,0x00,TransDataHeader,0x00,BloomFilterElement,NULL");
		return STATUS_SENT;
	}
	if (!ble_op || !prog || !service_name || !ble_role || !discovery_type) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "Invalid arguments");
		return INVALID_SEND_STATUS;
	}

	if ((strcasecmp(prog, "NAN") != 0)) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "Program %s not supported",
				prog);
		return INVALID_SEND_STATUS;
	}

	if (strcasecmp(ble_role, "seeker") != 0 &&
	    strcasecmp(ble_role, "provider") != 0 &&
	    strcasecmp(ble_role, "browser") != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "Invalid BLERole: %s",
				ble_role);
		return INVALID_SEND_STATUS;
	}

	if (strcasecmp(discovery_type, "active") != 0 &&
	    strcasecmp(discovery_type, "passive") != 0) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "Invalid DiscoveryType: %s",
				discovery_type);
		return INVALID_SEND_STATUS;
	}

	if (!M2Transmit)
		M2Transmit = "disable";

	argv[0] = "am";
	argv[1] = "start";
	argv[2] = "-n";
	argv[3] = "org.codeaurora.nanservicediscovery/org.codeaurora.nanservicediscovery.MainActivity";
	argv[4] = "--es";
	argv[5] = "service";
	argv[6] = (char *) service_name;
	argv[7] = "--es";
	argv[8] = "role";
	argv[9] = (char *) ble_role;
	argv[10] = "--es";
	argv[11] = "scantype";
	argv[12] = (char *) discovery_type;
	argv[13] = "--es";
	argv[14] = "M2Transmit";
	argv[15] = (char *) M2Transmit;
	argv[16] = NULL;

	pid = fork();
	if (pid == -1) {
		sigma_dut_print(dut, DUT_MSG_ERROR, "fork: %s",
				strerror(errno));
		return ERROR_SEND_STATUS;
	}

	if (pid == 0) {
		execv("/system/bin/am", argv);
		sigma_dut_print(dut, DUT_MSG_ERROR, "execv: %s",
				strerror(errno));
		exit(0);
		return ERROR_SEND_STATUS;
	}

	dut->nanservicediscoveryinprogress = 1;
#endif /* ANDROID */

	return SUCCESS_SEND_STATUS;
}



static enum sigma_cmd_result cmd_dev_start_test(struct sigma_dut *dut,
						struct sigma_conn *conn,
						struct sigma_cmd *cmd)
{
	const char *val;
	char buf[4096];
	char dir[4096];

	val = get_param(cmd, "Runtime_ID");
	if (val) {
		strncpy(dut->dev_start_test_runtime_id, val, sizeof(dut->dev_start_test_runtime_id));
		sigma_dut_print(dut, DUT_MSG_DEBUG, "Runtime_ID %s", dut->dev_start_test_runtime_id);
	} else {
		strncpy(dut->dev_start_test_runtime_id, "1234", sizeof(dut->dev_start_test_runtime_id));
		sigma_dut_print(dut, DUT_MSG_ERROR, "No runtime id, using default Runtime_ID %s", dut->dev_start_test_runtime_id);
	}

	if (dut->vendor_name)
		snprintf(dir, sizeof(dir), "%s_logs", dut->vendor_name);
	else
		snprintf(dir, sizeof(dir), "Qualcomm_logs");

	if (dut->log_file_dir) {
		snprintf(dir, sizeof(dir), "%s/%s", dut->log_file_dir, dut->vendor_name ? dut->vendor_name : "Qualcomm_logs");
	} else {
#ifdef ANDROID
		snprintf(dir, sizeof(dir), "/data/vendor/wifi/%s", dut->vendor_name ? dut->vendor_name : "Qualcomm_logs");
#else
		snprintf(dir, sizeof(dir), "/var/log/%s", dut->vendor_name ? dut->vendor_name : "Qualcomm_logs");
#endif /* ANDROID */
	}

	snprintf(buf, sizeof(buf), "%s/sigma_%s.txt", dir, dut->dev_start_test_runtime_id);
	dut->log_file_fd = fopen(buf, "w");
	if (!dut->log_file_fd)
		sigma_dut_print(dut, DUT_MSG_ERROR, "Failed to create sigma log file %s", buf);

	run_system_wrapper(dut, "rm -rf %s", dir);
	run_system_wrapper(dut, "mkdir -p %s", dir);
	run_system_wrapper(dut, "chmod -R 0666 %s", dir);
	run_system_wrapper(dut, "killall -9 cnss_diag_lite");
	run_system_wrapper(dut, "/usr/bin/cnss_diag_lite -c -x 31 > %s/cnss_diag_id_%s.txt &", dir, dut->dev_start_test_runtime_id);
#ifdef ANDROID
	run_system_wrapper(dut, "logcat V > %s/logcat_%s.txt &", dir, dut->dev_start_test_runtime_id);
#endif /* ANDROID */

	return SUCCESS_SEND_STATUS;
}


static enum sigma_cmd_result cmd_dev_stop_test(struct sigma_dut *dut,
					       struct sigma_conn *conn,
					       struct sigma_cmd *cmd)
{
	const char *val;
	char buf[4096];
	char out_file[4096];
	char dir[4096];

	val = get_param(cmd, "Runtime_ID");
	if (val && dut) {
		if (dut->log_file_fd) {
			fclose(dut->log_file_fd);
			dut->log_file_fd = NULL;
		}

		if (dut->vendor_name)
			snprintf(dir, sizeof(dir), "%s_logs", dut->vendor_name);
		else
			snprintf(dir, sizeof(dir), "Qualcomm_logs");

		if (dut->log_file_dir) {
			snprintf(dir, sizeof(dir), "%s/%s", dut->log_file_dir, dut->vendor_name ? dut->vendor_name : "Qualcomm_logs");
		} else {
#ifdef ANDROID
			snprintf(dir, sizeof(dir), "/data/vendor/wifi/%s", dut->vendor_name ? dut->vendor_name : "Qualcomm_logs");
#else
			snprintf(dir, sizeof(dir), "/var/log/%s", dut->vendor_name ? dut->vendor_name : "Qualcomm_logs");
#endif /* ANDROID */
		}

#ifndef ANDROID
		/*
		 * For Android all supplicant logs available in logcat,
		 * but for other platform, user has to redirect supplicat
		 * logs from stdout to below directory.
		 **/
		run_system_wrapper(dut, "cp -a /var/log/supplicant_log/* %s", dir);
#endif /* ANDROID */

		snprintf(out_file, sizeof(out_file), "%s_%s_%s.tar",
			 dut->vendor_name ? dut->vendor_name : "Qualcomm",
			 dut->model_name ? dut->model_name : "Unknown",
			 dut->dev_start_test_runtime_id);

		snprintf(buf, sizeof(buf), "tar -cvf %s/%s %s", dir, out_file, dir);
		if (run_system_wrapper(dut, buf) < 0) {
			sigma_dut_print(dut, DUT_MSG_ERROR, "Failed to create tar: %s", buf);
			return ERROR_SEND_STATUS;
		}

		val = get_param(cmd, "destpath");
		if (val)
			snprintf(buf, sizeof(buf), "tftp %s -c put %s/%s %s/%s", inet_ntoa(conn->addr.sin_addr), dir, out_file, val, out_file);
		else
			snprintf(buf, sizeof(buf), "tftp %s -c put %s/%s", inet_ntoa(conn->addr.sin_addr), dir, out_file);
		if (run_system_wrapper(dut, buf) < 0) {
			sigma_dut_print(dut, DUT_MSG_ERROR, "tftp file transfer Failed : %s", buf);
			return ERROR_SEND_STATUS;
		}
		sigma_dut_print(dut, DUT_MSG_DEBUG, "tftp file transfer : %s", buf);
		snprintf(buf, sizeof(buf), "filename, %s", out_file);
		send_resp(dut, conn, SIGMA_COMPLETE, buf);

		return SUCCESS_SEND_STATUS;
	}

	return ERROR_SEND_STATUS;
}


static enum sigma_cmd_result cmd_dev_get_log(struct sigma_dut *dut,
					     struct sigma_conn *conn,
					     struct sigma_cmd *cmd)
{
	return SUCCESS_SEND_STATUS;
}


static int req_intf(struct sigma_cmd *cmd)
{
	return get_param(cmd, "interface") == NULL ? -1 : 0;
}


static int req_role_svcname(struct sigma_cmd *cmd)
{
	if (!get_param(cmd, "BLERole"))
		 return -1;
	if (get_param(cmd, "BLEOp") && !get_param(cmd, "ServiceName"))
		return -1;
	return 0;
}


static int req_intf_prog(struct sigma_cmd *cmd)
{
	if (get_param(cmd, "interface") == NULL)
		return -1;
	if (get_param(cmd, "program") == NULL)
		return -1;
	return 0;
}


static int req_prog(struct sigma_cmd *cmd)
{
	if (get_param(cmd, "program") == NULL)
		return -1;
	return 0;
}


void dev_register_cmds(void)
{
	sigma_dut_reg_cmd("dev_send_frame", req_intf_prog, cmd_dev_send_frame);
	sigma_dut_reg_cmd("dev_set_parameter", req_intf_prog,
			  cmd_dev_set_parameter);
	sigma_dut_reg_cmd("dev_exec_action", req_prog,
			  cmd_dev_exec_action);
	sigma_dut_reg_cmd("dev_configure_ie", req_intf, cmd_dev_configure_ie);
	sigma_dut_reg_cmd("dev_start_test", NULL, cmd_dev_start_test);
	sigma_dut_reg_cmd("dev_stop_test", NULL, cmd_dev_stop_test);
	sigma_dut_reg_cmd("dev_get_log", NULL, cmd_dev_get_log);
	sigma_dut_reg_cmd("dev_ble_action", req_role_svcname,
			  cmd_dev_ble_action);
}
