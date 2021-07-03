/******************************************************************************
 *
 *  Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
 *  Copyright (C) 2014 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <sys/syslog.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <dlfcn.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <hardware/hardware.h>
#include <iostream>
#include <iomanip>
#include "Main.hpp"
#include <syslog.h>
#include "A2dp_Sink.hpp"
#include "HfpClient.hpp"
#include "pan/include/Pan.hpp"
#include "gatt/include/Gatt.hpp"
#include "Audio_Manager.hpp"

#include "utils.h"

#define LOGTAG  "MAIN "
#define LOCAL_SOCKET_NAME "/etc/bluetooth/btappsocket"

extern Gap *g_gap;
extern A2dp_Sink *pA2dpSink;
extern Pan *g_pan;
extern Gatt *g_gatt;
extern BT_Audio_Manager *pBTAM;
static BluetoothApp *g_bt_app = NULL;
extern ThreadInfo threadInfo[THREAD_ID_MAX];
extern Hfp_Client *pHfpClient;

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief main function
 *
 *
 *  This is main function of BT Application
 *
 * @param  argc
 * @param  *argv[]
 *
 */
int main (int argc, char *argv[]) {

    // initialize signal handler
    signal(SIGINT, SignalHandler);

    ThreadInfo *main_thread = &threadInfo[THREAD_ID_MAIN];
    openlog ("bt-app", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);

    main_thread->thread_id = thread_new (main_thread->thread_name);
    if (main_thread->thread_id) {
        BtEvent *event = new BtEvent;
        event->event_id = MAIN_API_INIT;
        ALOGV (LOGTAG " Posting init to Main thread\n");
        PostMessage (THREAD_ID_MAIN, event);

        // wait for Main thread to exit
        thread_join (main_thread->thread_id);
        thread_free (main_thread->thread_id);
    }
    closelog ();
    return 0;
}

static bool HandleUserInput (int *cmd_id, char input_args[][COMMAND_ARG_SIZE],
                                                          MenuType menu_type) {
    char user_input[COMMAND_SIZE] = {'\0'};
    int index = 0 , found_index = -1, num_cmds;
    char *temp_arg = NULL;
    char delim[] = " ";
    int param_count = 0;
    bool status = false;
    int max_param = 0;
    UserMenuList *menu = NULL;

    // validate the input string
    if(((fgets (user_input, sizeof (user_input), stdin)) == NULL) ||
       (user_input[0] == '\n')) {
        return status;
    }
    // remove trialing \n character
    user_input[strlen(user_input) - 1] = '\0';

    // According to the current menu assign Command menu
    switch(menu_type) {
        case GAP_MENU:
            menu = &GapMenu[0];
            num_cmds  = NO_OF_COMMANDS(GapMenu);
            break;
        case PAN_MENU:
            menu = &PanMenu[0];
            num_cmds  = NO_OF_COMMANDS(PanMenu);
            break;
        case TEST_MENU:
            menu = &TestMenu[0];
            num_cmds  = NO_OF_COMMANDS(TestMenu);
            break;
        case A2DP_SINK_MENU:
            menu = &A2dpSinkMenu[0];
            num_cmds  = NO_OF_COMMANDS(A2dpSinkMenu);
            break;
        case HFP_CLIENT_MENU:
            menu = &HfpClientMenu[0];
            num_cmds  = NO_OF_COMMANDS(HfpClientMenu);
            break;
        case MAIN_MENU:
        // fallback to default main menu
        default:
            menu = &MainMenu[0];
            num_cmds  = NO_OF_COMMANDS(MainMenu);
            break;
    }

    if ( (temp_arg = strtok(user_input, delim)) != NULL ) {
        // find out the command name
        for (index = 0; index < num_cmds; index++) {
            if(!strcasecmp (menu[index].cmd_name, temp_arg)) {
                *cmd_id = menu[index].cmd_id;
                found_index = index;
                strncpy(input_args[param_count], temp_arg, COMMAND_ARG_SIZE - 1);
                input_args[param_count++][COMMAND_ARG_SIZE - 1] = '\0';
                break;
            }
        }

        // validate the command parameters
        if (found_index != -1 ) {
            max_param = menu[found_index].max_param;
            while ((temp_arg = strtok(NULL, delim)) &&
                    (param_count < max_param + 1)) {
                strncpy(input_args[param_count], temp_arg, COMMAND_ARG_SIZE - 1);
                input_args[param_count++][COMMAND_ARG_SIZE - 1] = '\0';
            }

            // consider command as other param
            if(param_count == max_param + 1) {
                if(temp_arg != NULL) {
                    fprintf( stdout, " Maximum params reached\n");
                    fprintf( stdout, " Refer help: %s\n", menu[found_index].cmd_help);
                } else {
                    status = true;
                }
            } else if(param_count < max_param + 1) {
                fprintf( stdout, " Missing required parameters\n");
                fprintf( stdout, " Refer help: %s\n", menu[found_index].cmd_help);
            }
        } else {
            // to handle the paring inputs
            if(temp_arg != NULL) {
                strncpy(input_args[param_count], temp_arg, COMMAND_ARG_SIZE - 1);
                input_args[param_count++][COMMAND_ARG_SIZE - 1] = '\0';
            }
        }
    }
    return status;
}

static void DisplayMenu(MenuType menu_type) {

    UserMenuList *menu = NULL;
    int index = 0, num_cmds = 0;

    switch(menu_type) {
        case GAP_MENU:
            menu = &GapMenu[0];
            num_cmds  = NO_OF_COMMANDS(GapMenu);
            break;
        case PAN_MENU:
            menu = &PanMenu[0];
            num_cmds  = NO_OF_COMMANDS(PanMenu);
            break;
        case TEST_MENU:
            menu = &TestMenu[0];
            num_cmds  = NO_OF_COMMANDS(TestMenu);
            break;
        case MAIN_MENU:
            menu = &MainMenu[0];
            num_cmds  = NO_OF_COMMANDS(MainMenu);
            break;
        case A2DP_SINK_MENU:
            menu = &A2dpSinkMenu[0];
            num_cmds  = NO_OF_COMMANDS(A2dpSinkMenu);
            break;
        case HFP_CLIENT_MENU:
            menu = &HfpClientMenu[0];
            num_cmds  = NO_OF_COMMANDS(HfpClientMenu);
            break;
    }
    fprintf (stdout, " \n***************** Menu *******************\n");
    for (index = 0; index < num_cmds; index++)
        fprintf (stdout, "\t %s \n",  menu[index].cmd_help);
    fprintf (stdout, " ******************************************\n");
}

static void SignalHandler(int sig) {
    signal(SIGINT, SIG_IGN);
    ExitHandler();
}

static void ExitHandler(void) {

    // post the disable message to GAP incase BT is on
    if (g_bt_app->bt_state == BT_STATE_ON) {
        BtEvent *event = new BtEvent;
        event->event_id = GAP_API_DISABLE;
        PostMessage (THREAD_ID_GAP, event);
        sleep(1);
    }

    // TODO to wait for complete turn off before proceeding

    if (g_bt_app) {
        BtEvent *event = new BtEvent;
        event->event_id = MAIN_API_DEINIT;
        g_bt_app->ProcessEvent (event);
        delete event;
        delete g_bt_app;
        g_bt_app = NULL;
    }

    // stop the reactor for self exit of main thread
    reactor_stop (thread_get_reactor (threadInfo[THREAD_ID_MAIN].thread_id));
}

static void HandleA2dpSinkCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]) {
    ALOGD(LOGTAG, "HandleA2DPSinkCommand cmd_id = %d", cmd_id);
    BtEvent *event = NULL;
    switch (cmd_id) {
        case CONNECT:
            event = new BtEvent;
            event->a2dpSinkEvent.event_id = A2DP_SINK_API_CONNECT_REQ;
            string_to_bdaddr(user_cmd[ONE_PARAM], &event->a2dpSinkEvent.bd_addr);
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case DISCONNECT:
            event = new BtEvent;
            event->a2dpSinkEvent.event_id = A2DP_SINK_API_DISCONNECT_REQ;
            string_to_bdaddr(user_cmd[ONE_PARAM], &event->a2dpSinkEvent.bd_addr);
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case PLAY:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_PLAY;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case PAUSE:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_PAUSE;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case STOP:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_STOP;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case FASTFORWARD:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_FF;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case REWIND:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_REWIND;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case FORWARD:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_FORWARD;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case BACKWARD:
            event = new BtEvent;
            event->avrcpCtrlEvent.event_id = AVRCP_CTRL_PASS_THRU_CMD_REQ;
            event->avrcpCtrlEvent.key_id = CMD_ID_BACKWARD;
            PostMessage (THREAD_ID_A2DP_SINK, event);
            break;
        case BACK_TO_MAIN:
            menu_type = MAIN_MENU;
            DisplayMenu(menu_type);
            break;
    }
}

static void HandleHfpClientCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]) {
    ALOGD(LOGTAG, "HandleHfpClientCommand cmd_id = %d", cmd_id);
    BtEvent *event = NULL;
    switch (cmd_id) {
        case CONNECT:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_CONNECT_REQ;
            string_to_bdaddr(user_cmd[ONE_PARAM], &event->hfp_client_event.bd_addr);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case DISCONNECT:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_DISCONNECT_REQ;
            string_to_bdaddr(user_cmd[ONE_PARAM], &event->hfp_client_event.bd_addr);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case CREATE_SCO_CONN:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_CONNECT_AUDIO_REQ;
            string_to_bdaddr(user_cmd[ONE_PARAM], &event->hfp_client_event.bd_addr);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case DESTROY_SCO_CONN:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_DISCONNECT_AUDIO_REQ;
            string_to_bdaddr(user_cmd[ONE_PARAM], &event->hfp_client_event.bd_addr);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case ACCEPT_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_ACCEPT_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case REJECT_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_REJECT_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case END_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_END_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case HOLD_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_HOLD_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case RELEASE_HELD_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_RELEASE_HELD_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case RELEASE_ACTIVE_ACCEPT_WAITING_OR_HELD_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_RELEASE_ACTIVE_ACCEPT_WAITING_OR_HELD_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case SWAP_CALLS:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_SWAP_CALLS_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case ADD_HELD_CALL_TO_CONF:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_ADD_HELD_CALL_TO_CONF_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case RELEASE_SPECIFIED_ACTIVE_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_RELEASE_SPECIFIED_ACTIVE_CALL_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case PRIVATE_CONSULTATION_MODE:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_PRIVATE_CONSULTATION_MODE_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case PUT_INCOMING_CALL_ON_HOLD:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_PUT_INCOMING_CALL_ON_HOLD_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case ACCEPT_HELD_INCOMING_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_ACCEPT_HELD_INCOMING_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case REJECT_HELD_INCOMING_CALL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_REJECT_HELD_INCOMING_CALL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case DIAL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_DIAL_REQ;
            strncpy(event->hfp_client_event.str, user_cmd[ONE_PARAM], 20);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case REDIAL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_REDIAL_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case DIAL_MEMORY:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_DIAL_MEMORY_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case START_VR:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_START_VR_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case STOP_VR:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_STOP_VR_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case CALL_ACTION:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_CALL_ACTION_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            event->hfp_client_event.arg2 = atoi(user_cmd[TWO_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case QUERY_CURRENT_CALLS:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_QUERY_CURRENT_CALLS_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case QUERY_OPERATOR_NAME:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_QUERY_OPERATOR_NAME_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case QUERY_SUBSCRIBER_INFO:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_QUERY_SUBSCRIBER_INFO_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case SCO_VOL_CTRL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_SCO_VOL_CTRL_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            event->hfp_client_event.arg2 = atoi(user_cmd[TWO_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case MIC_VOL_CTRL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_MIC_VOL_CTRL_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case SPK_VOL_CTRL:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_SPK_VOL_CTRL_REQ;
            event->hfp_client_event.arg1 = atoi(user_cmd[ONE_PARAM]);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case SEND_DTMF:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_SEND_DTMF_REQ;
            strncpy(event->hfp_client_event.str, user_cmd[ONE_PARAM], 20);
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case DISABLE_NREC_ON_AG:
            event = new BtEvent;
            event->hfp_client_event.event_id = HFP_CLIENT_API_DISABLE_NREC_ON_AG_REQ;
            PostMessage (THREAD_ID_HFP_CLIENT, event);
            break;
        case BACK_TO_MAIN:
            menu_type = MAIN_MENU;
            DisplayMenu(menu_type);
            break;
    }
}

static void HandleMainCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]) {

    switch (cmd_id) {
        case GAP_OPTION:
            menu_type = GAP_MENU;
            DisplayMenu(menu_type);
            break;
        case PAN_OPTION:
            menu_type = PAN_MENU;
            DisplayMenu(menu_type);
            break;
        case TEST_MODE:
            menu_type = TEST_MENU;
            DisplayMenu(menu_type);
            break;
        case A2DP_SINK:
            menu_type = A2DP_SINK_MENU;
            DisplayMenu(menu_type);
            break;

        case HFP_CLIENT:
            menu_type = HFP_CLIENT_MENU;
            DisplayMenu(menu_type);
            break;

        case MAIN_EXIT:
            ALOGV (LOGTAG " Self exit of Main thread");
            ExitHandler();
            break;
         default:
            ALOGV (LOGTAG " Command not handled");
            break;
    }
}

static void HandleTestCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]) {

    long num;
    char *end;
    int index = 0;
    switch (cmd_id) {
        case TEST_ON_OFF:
            if ( user_cmd[ONE_PARAM][0] != '\0') {
                errno = 0;
                num = strtol(user_cmd[ONE_PARAM], &end, 0);
                if (*end != '\0' || errno != 0 || num < INT_MIN || num > INT_MAX){
                    fprintf( stdout, " Enter numeric Value\n");
                    break;
                }
                for( index = 0; index < (int)num; index++){
                    BtEvent *event_on = new BtEvent;
                    event_on->event_id = GAP_API_ENABLE;
                    PostMessage (THREAD_ID_GAP, event_on);
                    sleep(2);
                    BtEvent *event_off = new BtEvent;
                    event_off->event_id = GAP_API_DISABLE;
                    PostMessage (THREAD_ID_GAP, event_off);
                    sleep(2);
                }
            }
            fprintf( stdout, "Currently not Handled %ld \n", num);
            break;

        case RSP_INIT:
            if ((g_bt_app->bt_state == BT_STATE_ON)) {
                fprintf( stdout, "ENABLE RSP\n");
                if (g_gatt) g_gatt->rsp->EnableRSP();
            } else {
                fprintf( stdout, "BT is in OFF State now \n");
            }
            break;

        case RSP_START:
            if ((g_bt_app->bt_state == BT_STATE_ON)) {
                fprintf( stdout, "(Re)start Advertisement \n");
                if (g_gatt) g_gatt->rsp->StartAdvertisement();
            } else {
                fprintf( stdout, "BT is in OFF State now \n");
            }
            break;

        case BACK_TO_MAIN:
            menu_type = MAIN_MENU;
            DisplayMenu(menu_type);
            break;
        default:
            ALOGV (LOGTAG " Command not handled");
            break;
    }
}

static void HandleGapCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]) {
    BtEvent *event = NULL;

    switch (cmd_id) {
        case BACK_TO_MAIN:
            menu_type = MAIN_MENU;
            DisplayMenu(menu_type);
            break;

        case BT_ENABLE:
            if ((g_bt_app->status.enable_cmd != COMMAND_INPROGRESS) &&
                (g_bt_app->bt_state == BT_STATE_OFF)) {

                g_bt_app->status.enable_cmd = COMMAND_INPROGRESS;
                BtEvent *event = new BtEvent;

                event->event_id = GAP_API_ENABLE;
                ALOGV (LOGTAG " Posting BT enable to GAP thread");
                PostMessage (THREAD_ID_GAP, event);
            } else if ( g_bt_app->status.enable_cmd == COMMAND_INPROGRESS ) {
                fprintf( stdout, "BT enable is already in process\n");
            } else {
                fprintf( stdout, "Currently BT is already ON\n");
            }
            break;

        case BT_DISABLE:

            if ((g_bt_app->status.disable_cmd != COMMAND_INPROGRESS) &&
                                (g_bt_app->bt_state == BT_STATE_ON)) {

                g_bt_app->status.disable_cmd = COMMAND_INPROGRESS;
                event = new BtEvent;
                event->event_id = GAP_API_DISABLE;
                ALOGV (LOGTAG " Posting disable to GAP thread");
                PostMessage (THREAD_ID_GAP, event);
            } else if (g_bt_app->status.disable_cmd == COMMAND_INPROGRESS) {
                fprintf( stdout, " disable command is already in process\n");
            } else {
                fprintf( stdout, "Currently BT is already OFF\n");
            }
            break;

        case START_ENQUIRY:

            if ((g_bt_app->status.enquiry_cmd != COMMAND_INPROGRESS) &&
                                (g_bt_app->bt_state == BT_STATE_ON)) {

                g_bt_app->status.enquiry_cmd = COMMAND_INPROGRESS;
                event = new BtEvent;
                event->event_id = GAP_API_START_INQUIRY;
                ALOGV (LOGTAG " Posting inquiry to GAP thread");
                PostMessage (THREAD_ID_GAP, event);

            } else if (g_bt_app->status.enquiry_cmd == COMMAND_INPROGRESS) {
                fprintf( stdout, " The inquiry is already in process\n");

            } else {
                fprintf( stdout, "currently BT is OFF\n");
            }
            break;

        case CANCEL_ENQUIRY:

            if ((g_bt_app->status.stop_enquiry_cmd != COMMAND_INPROGRESS) &&
                (g_bt_app->bt_discovery_state == BT_DISCOVERY_STARTED) &&
                        (g_bt_app->bt_state == BT_STATE_ON)) {
                g_bt_app->status.stop_enquiry_cmd = COMMAND_INPROGRESS;
                event = new BtEvent;
                event->event_id = GAP_API_STOP_INQUIRY;
                ALOGV (LOGTAG " Posting stop inquiry to GAP thread");
                PostMessage (THREAD_ID_GAP, event);

            } else if (g_bt_app->status.stop_enquiry_cmd == COMMAND_INPROGRESS) {
                fprintf( stdout, " The stop inquiry is already in process\n");

            } else if (g_bt_app->bt_state == BT_STATE_OFF) {
                fprintf( stdout, "currently BT is OFF\n");

            } else if (g_bt_app->bt_discovery_state != BT_DISCOVERY_STARTED) {
                fprintf( stdout,"Inquiry is not started, ignoring the stop inquiry\n");
            }
            break;

        case START_PAIR:
            if ((g_bt_app->status.pairing_cmd != COMMAND_INPROGRESS) &&
                (g_bt_app->bt_state == BT_STATE_ON)) {
                if (string_is_bdaddr(user_cmd[ONE_PARAM])) {
                    g_bt_app->status.pairing_cmd = COMMAND_INPROGRESS;
                    event = new BtEvent;
                    event->event_id = GAP_API_CREATE_BOND;
                    string_to_bdaddr(user_cmd[ONE_PARAM], &event->bond_device.bd_addr);
                    PostMessage (THREAD_ID_GAP, event);
                } else {
                 fprintf( stdout, " BD address is NULL/Invalid \n");
                }
            } else if (g_bt_app->status.pairing_cmd == COMMAND_INPROGRESS) {
                fprintf( stdout, " Pairing is already in process\n");
            } else {
                fprintf( stdout, " Currently BT is OFF\n");
            }
            break;

        case UNPAIR:
            if ( g_bt_app->bt_state == BT_STATE_ON ) {
                if (string_is_bdaddr(user_cmd[ONE_PARAM])) {
                    bt_bdaddr_t bd_addr;
                    string_to_bdaddr(user_cmd[ONE_PARAM], &bd_addr);
                    g_bt_app->HandleUnPair(bd_addr);
                } else {
                    fprintf( stdout, " BD address is NULL/Invalid \n");
                }
            } else {
                fprintf( stdout, " Currently BT is OFF\n");
            }
            break;

        case INQUIRY_LIST:
            if (!g_bt_app->inquiry_list.empty()) {
                g_bt_app->PrintInquiryList();
            } else {
                fprintf( stdout, " Empty Inquiry list\n");
            }
            break;

        case BONDED_LIST:
            if (!g_bt_app->bonded_devices.empty()) {
                g_bt_app->PrintBondedDeviceList();
            } else {
                fprintf( stdout, " Empty bonded list\n");
            }
            break;

        case GET_BT_STATE:
           if ( g_bt_app->GetState() == BT_STATE_ON )
               fprintf( stdout, "ON\n" );
            else if ( g_bt_app->GetState() == BT_STATE_OFF)
                fprintf( stdout, "OFF\n");
            break;

        case GET_BT_NAME:
            if ( g_bt_app->GetState() == BT_STATE_ON ) {
                fprintf(stdout, "BT Name : %s\n", g_gap->GetBtName());
            } else {
                fprintf( stdout, "No Name due to BT is OFF\n");
            }
            break;

        case GET_BT_ADDR:
            if ( g_bt_app->GetState() == BT_STATE_ON ) {
                bdstr_t bd_str;
                bt_bdaddr_t *bd_addr = g_gap->GetBtAddress();
                bdaddr_to_string(bd_addr, &bd_str[0], sizeof(bd_str));
                std::string deviceAddress(bd_str);
                std::cout << " BT Address :" << deviceAddress << std::endl;
            } else {
                fprintf( stdout, "No Addr due to BT is OFF\n");
            }
            break;

        case SET_BT_NAME:
            if ( g_bt_app->GetState() == BT_STATE_ON ) {
                if (strlen(user_cmd[ONE_PARAM]) < BTM_MAX_LOC_BD_NAME_LEN &&
                    (user_cmd[ONE_PARAM] != NULL) ) {
                    bt_bdname_t bd_name;
                    event = new BtEvent;
                    event->event_id = GAP_API_SET_BDNAME;
                    event->set_device_name_event.prop.type = BT_PROPERTY_BDNAME;
                    strcpy((char*)&bd_name.name[0],user_cmd[ONE_PARAM]);
                    event->set_device_name_event.prop.val = &bd_name;
                    event->set_device_name_event.prop.len = strlen((char*)bd_name.name);
                    PostMessage (THREAD_ID_GAP, event);
                } else {
                 fprintf( stdout, " BD Name is NULL/more than required legnth\n");
                }
            } else {
                fprintf( stdout, " Currently BT is OFF\n");
            }
            break;

        default:
            ALOGV (LOGTAG " Command not handled");
            break;
    }
}

static void HandlePanCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]) {

    long num;
    char *end;
    int index = 0;

    switch (cmd_id) {
        case DISCONNECT:
            if ((g_bt_app->bt_state == BT_STATE_ON)) {
                if (string_is_bdaddr(user_cmd[ONE_PARAM])) {
                    BtEvent *event = new BtEvent;
                    event->event_id = PAN_EVENT_DEVICE_DISCONNECT_REQ;
                    string_to_bdaddr(user_cmd[ONE_PARAM],
                            &event->pan_device_disconnect_event.bd_addr);
                    PostMessage (THREAD_ID_PAN, event);
                } else {
                    fprintf(stdout, " BD address is NULL/Invalid ");
                }
            } else {
                fprintf(stdout, " Currently BT is in OFF state");
            }
            break;

        case CONNECTED_LIST:
            if ((g_bt_app->bt_state == BT_STATE_ON)) {
                BtEvent *event = new BtEvent;
                event->event_id = PAN_EVENT_DEVICE_CONNECTED_LIST_REQ;
                PostMessage (THREAD_ID_PAN, event);
            } else {
                fprintf(stdout," Currently BT is in OFF state");
            }
            break;

        case SET_TETHERING:

            if ((g_bt_app->bt_state == BT_STATE_ON)) {
                bool is_tethering_enable;

                if (!strcasecmp (user_cmd[ONE_PARAM], "true")) {
                    is_tethering_enable = true;
                } else if (!strcasecmp (user_cmd[ONE_PARAM], "false")) {
                    is_tethering_enable = false;
                } else {
                    fprintf(stdout, " Wrong option selected\n");
                    return;
                }

                BtEvent *event = new BtEvent;
                event->event_id = PAN_EVENT_SET_TETHERING_REQ;
                event->pan_set_tethering_event.is_tethering_on = is_tethering_enable;
                PostMessage (THREAD_ID_PAN, event);
            } else {
                fprintf(stdout, " Currently BT is in OFF state");
            }
            break;

        case BACK_TO_MAIN:
            menu_type = MAIN_MENU;
            DisplayMenu(menu_type);
            break;

        default:
        ALOGV (LOGTAG " Command not handled: %d", cmd_id);
        break;
    }
}

void BtSocketDataHandler (void *context) {
    char ipc_msg[BT_IPC_MSG_LEN]  = {0};
    int len;
    if(g_bt_app->client_socket_ != -1) {
        len = recv(g_bt_app->client_socket_, ipc_msg, BT_IPC_MSG_LEN, 0);

        if (len <= 0) {
            ALOGE("Not able to receive msg to remote dev: %s", strerror(errno));
            reactor_unregister (g_bt_app->accept_reactor_);
            g_bt_app->client_socket_ = -1;
        } else if(len == BT_IPC_MSG_LEN) {
            BtEvent *event = new BtEvent;
            event->event_id = SKT_API_IPC_MSG_READ;
            event->bt_ipc_msg_event.ipc_msg.type = ipc_msg[0];
            event->bt_ipc_msg_event.ipc_msg.status = ipc_msg[1];

            switch (event->bt_ipc_msg_event.ipc_msg.type) {
                /*fall through for PAN IPC message*/
                case BT_IPC_ENABLE_TETHERING:
                case BT_IPC_DISABLE_TETHERING:
                    ALOGV (LOGTAG "  Posting IPC_MSG to PAN thread");
                    PostMessage (THREAD_ID_PAN, event);
                    break;
                case BT_IPC_REMOTE_START_WLAN:
                    ALOGV (LOGTAG "  Posting IPC_MSG to GATT thread");
                    PostMessage (THREAD_ID_GATT, event);
                    break;
                default:
                    delete event;
                break;
            }
        }
    }
}

void BtSocketListenHandler (void *context) {
    struct sockaddr_un cliaddr;
    int length;

    if(g_bt_app->client_socket_ == -1) {
        g_bt_app->client_socket_ = accept(g_bt_app->listen_socket_local_,
            (struct sockaddr*) &cliaddr, ( socklen_t *) &length);
        if (g_bt_app->client_socket_ == -1) {
            ALOGE (LOGTAG "%s error accepting LOCAL socket: %s",
                        __func__, strerror(errno));
        } else {
            g_bt_app->accept_reactor_ = reactor_register
                (thread_get_reactor (threadInfo[THREAD_ID_MAIN].thread_id),
                g_bt_app->client_socket_, NULL, BtSocketDataHandler, NULL);
        }
    } else {
        ALOGI (LOGTAG " Accepting and closing the next connection .\n");
        int accept_socket = accept(g_bt_app->listen_socket_local_,
            (struct sockaddr*) &cliaddr, ( socklen_t *) &length);
        if(accept_socket)
            close(accept_socket);
    }
}

static void BtCmdHandler (void *context) {

    int cmd_id = -1;
    char user_cmd[MAX_ARGUMENTS][COMMAND_ARG_SIZE];
    int index = 0;
    memset( (void *) user_cmd, '\0', sizeof(user_cmd));

    if (HandleUserInput (&cmd_id, user_cmd, menu_type)) {
        switch(menu_type) {
            case GAP_MENU:
                HandleGapCommand(cmd_id,user_cmd);
                break;
            case PAN_MENU:
                HandlePanCommand(cmd_id, user_cmd);
                break;
            case TEST_MENU:
                HandleTestCommand(cmd_id, user_cmd);
                break;
            case MAIN_MENU:
                HandleMainCommand(cmd_id,user_cmd );
                break;
            case A2DP_SINK_MENU:
                HandleA2dpSinkCommand(cmd_id,user_cmd );
                break;
            case HFP_CLIENT_MENU:
                HandleHfpClientCommand(cmd_id,user_cmd );
                break;
        }
    } else if (g_bt_app->ssp_notification && user_cmd[0][0] &&
                        g_bt_app->HandleSspInput(user_cmd)) {
        // validate the user input for SSP
        g_bt_app->ssp_notification = false;
    } else if (g_bt_app->pin_notification && user_cmd[0][0] &&
                        g_bt_app->HandlePinInput(user_cmd)) {
        // validate the user input for PIN
        g_bt_app->pin_notification = false;
    } else {
        fprintf( stdout, " Wrong option selected\n");
        DisplayMenu(menu_type);
        // TODO print the given input string
        return;
    }
}

void BtMainMsgHandler (void *context) {

    BtEvent *event = NULL;
    if (!context) {
        ALOGI (LOGTAG " Msg is null, return.\n");
        return;
    }
    event = (BtEvent *) context;

    switch (event->event_id) {
        case SKT_API_IPC_MSG_WRITE:
            ALOGV (LOGTAG "client_socket: %d", g_bt_app->client_socket_);
            if(g_bt_app->client_socket_ != -1) {
                int len;
                if((len = send(g_bt_app->client_socket_, &(event->bt_ipc_msg_event.ipc_msg),
                    BT_IPC_MSG_LEN, 0)) < 0) {
                    reactor_unregister (g_bt_app->accept_reactor_);
                    g_bt_app->client_socket_ = -1;
                    ALOGE (LOGTAG "Local socket send fail %s", strerror(errno));
                }
                ALOGV (LOGTAG "sent %d bytes", len);
            }
            delete event;
            break;

        case MAIN_API_INIT:
            if (!g_bt_app)
                g_bt_app = new BluetoothApp();
        // fallback to default handler
        default:
            if (g_bt_app)
                g_bt_app->ProcessEvent ((BtEvent *) context);
            delete event;
            break;
    }
}

#ifdef __cplusplus
}
#endif

bool BluetoothApp :: HandlePinInput(char user_cmd[][COMMAND_ARG_SIZE]) {
    BtEvent *bt_event = new BtEvent;

    if (pin_reply.secure  == true ) {
        if ( strlen(user_cmd[ZERO_PARAM]) != 16){
            fprintf(stdout, " Minimum 16 digit pin required\n");
            return false;
        }
    } else if(strlen(user_cmd[ZERO_PARAM]) >16 ) {
           return false;
    }

    memset(&pin_reply.pincode, 0, sizeof(bt_pin_code_t));
    memcpy(&pin_reply.pincode.pin, user_cmd[ZERO_PARAM],
                        strlen(user_cmd[ZERO_PARAM]));
    memcpy(&bt_event->pin_reply_event.bd_addr, &pin_reply.bd_addr,
                                            sizeof(bt_bdaddr_t));
    bt_event->pin_reply_event.pin_len = strlen(user_cmd[ZERO_PARAM]);
    memcpy(&bt_event->pin_reply_event.bd_name, &pin_reply.bd_name,
                                        sizeof(bt_bdname_t));
    memcpy(&bt_event->pin_reply_event.pincode.pin, &pin_reply.pincode.pin,
                                        sizeof(bt_pin_code_t));
    bt_event->event_id = GAP_API_PIN_REPLY;
    PostMessage (THREAD_ID_GAP, bt_event);
    return true;
}


bool BluetoothApp :: HandleSspInput(char user_cmd[][COMMAND_ARG_SIZE]) {


    BtEvent *bt_event = new BtEvent;
    if (!strcasecmp (user_cmd[ZERO_PARAM], "yes")) {
        ssp_data.accept = true;
    }
    else if (!strcasecmp (user_cmd[ZERO_PARAM], "no")) {
        ssp_data.accept = false;
    } else {
        fprintf( stdout, " Wrong option selected\n");
        return false;
    }

    memcpy(&bt_event->ssp_reply_event.bd_addr, &ssp_data.bd_addr,
                                            sizeof(bt_bdaddr_t));
    memcpy(&bt_event->ssp_reply_event.bd_name, &ssp_data.bd_name,
                                        sizeof(bt_bdname_t));
    bt_event->ssp_reply_event.cod = ssp_data.cod;
    bt_event->ssp_reply_event.pairing_variant = ssp_data.pairing_variant;
    bt_event->ssp_reply_event.pass_key = ssp_data.pass_key;
    bt_event->ssp_reply_event.accept = ssp_data.accept;
    bt_event->event_id = GAP_API_SSP_REPLY;
    PostMessage (THREAD_ID_GAP, bt_event);
    return true;
}

void BluetoothApp :: ProcessEvent (BtEvent * event) {

    ALOGD (LOGTAG " Processing event %d", event->event_id);
    const uint8_t *ptr = NULL;

    switch (event->event_id) {
        case MAIN_API_INIT:
            InitHandler();
            break;

        case MAIN_API_DEINIT:
            DeInitHandler();
            break;

        case MAIN_EVENT_ENABLED:
            bt_state = event->state_event.status;
            if (event->state_event.status == BT_STATE_OFF) {
                fprintf(stdout," Error in Enabling BT\n");
            } else {
               fprintf(stdout," BT State is ON\n");
            }
            status.enable_cmd = COMMAND_COMPLETE;
            break;

        case MAIN_EVENT_DISABLED:
            bt_state = event->state_event.status;
            if (event->state_event.status == BT_STATE_ON) {
                fprintf(stdout, " Error in disabling BT\n");
            } else {
                // clear the inquiry related cmds
                status.enquiry_cmd = COMMAND_COMPLETE;
                status.stop_enquiry_cmd = COMMAND_COMPLETE;
                bt_discovery_state = BT_DISCOVERY_STOPPED;
                // clearing bond_devices list and inquiry_list
                bonded_devices.clear();
                inquiry_list.clear();
               fprintf(stdout, " BT State is OFF\n");
            }
            status.disable_cmd = COMMAND_COMPLETE;
            break;

        case MAIN_EVENT_ACL_CONNECTED:
            ALOGD (LOGTAG " MAIN_EVENT_ACL_CONNECTED\n");
            break;

        case MAIN_EVENT_ACL_DISCONNECTED:
            ALOGD (LOGTAG " MAIN_EVENT_ACL_DISCONNECTED\n");
            break;

        case MAIN_EVENT_INQUIRY_STATUS:
            if (event->discovery_state_event.state == BT_DISCOVERY_STARTED) {
                fprintf(stdout, " Inquiry Started\n");
            } else if (event->discovery_state_event.state == BT_DISCOVERY_STOPPED) {
                if ( status.enquiry_cmd == COMMAND_INPROGRESS) {
                    if ((bt_discovery_state == BT_DISCOVERY_STARTED) &&
                       (status.stop_enquiry_cmd != COMMAND_INPROGRESS))
                        fprintf(stdout, " Inquiry Stopped automatically\n");
                    else if (bt_discovery_state == BT_DISCOVERY_STOPPED)
                        fprintf(stdout, " Unable to start Inquiry\n");
                    status.enquiry_cmd = COMMAND_COMPLETE;
                }
                if (status.stop_enquiry_cmd == COMMAND_INPROGRESS) {
                    status.stop_enquiry_cmd = COMMAND_COMPLETE;
                    fprintf(stdout," Inquiry Stopped due to user input\n");
                }
            }
            bt_discovery_state = event->discovery_state_event.state;
            break;

        case MAIN_EVENT_DEVICE_FOUND:
            fprintf(stdout, "Device Found details: \n");
            AddFoundedDevice(event->device_found_event.remoteDevice.name,
                                    event->device_found_event.remoteDevice.address);
            ptr = (event->device_found_event.remoteDevice.address.address);
            fprintf(stdout,"Found device Addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
                                ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
            fprintf(stdout, "Found device Name: %s\n", event->device_found_event.
                                                    remoteDevice.name);
            fprintf(stdout, "Device class is: %d\n", event->device_found_event.
                                        remoteDevice.bluetooth_class);
            break;

        case MAIN_EVENT_BOND_STATE: {
            std::string bd_name((const char*)event->bond_state_event.bd_name.name);
            HandleBondState(event->bond_state_event.state,
                                    event->bond_state_event.bd_addr, bd_name);
            }
            break;

        case MAIN_EVENT_SSP_REQUEST:
            memcpy(&ssp_data.bd_addr, &event->ssp_request_event.bd_addr,
                                            sizeof(bt_bdaddr_t));
            memcpy(&ssp_data.bd_name, &event->ssp_request_event.bd_name,
                                                    sizeof(bt_bdname_t));
            ssp_data.cod = event->ssp_request_event.cod;
            ssp_data.pairing_variant = event->ssp_request_event.pairing_variant;
            ssp_data.pass_key = event->ssp_request_event.pass_key;
            // instruct the cmd handler to treat the next inputs for SSP
            fprintf(stdout, "\n*************************************************");
            fprintf(stdout, "\n BT pairing request::Device %s::Pairing Code:: %d",
                                    ssp_data.bd_name.name, ssp_data.pass_key);
            fprintf(stdout, "\n*************************************************\n");
            fprintf(stdout, " ** Please enter yes / no **\n");
            ssp_notification = true;
            break;

        case MAIN_EVENT_PIN_REQUEST:

            memcpy(&pin_reply.bd_addr, &event->pin_request_event.bd_addr,
                                            sizeof(bt_bdaddr_t));
            memcpy(&pin_reply.bd_name, &event->pin_request_event.bd_name,
                                            sizeof(bt_bdname_t));
            fprintf(stdout, "\n*************************************************");
            fprintf(stdout, "\n BT Legacy pairing request::Device %s::",
                                    pin_reply.bd_name.name);
            fprintf(stdout, "\n*************************************************\n");
            fprintf(stdout, " ** Please enter valid PIN key **\n");
            pin_reply.secure = event->pin_request_event.secure;
            // instruct the cmd handler to treat the next inputs for PIN
            pin_notification = true;
            break;

        default:
            ALOGD (LOGTAG " Default Case");
            break;
    }
}

void BluetoothApp:: HandleBondState(bt_bond_state_t new_state, const bt_bdaddr_t
                                        bd_addr, std::string bd_name ) {
    std::map<std::string, std::string>::iterator it;
    bdstr_t bd_str;
    bdaddr_to_string(&bd_addr, &bd_str[0], sizeof(bd_str));
    std::string deviceAddress(bd_str);
    it = bonded_devices.find(deviceAddress);

    if(new_state == BT_BOND_STATE_BONDED) {
        if (it == bonded_devices.end()) {
            bonded_devices[deviceAddress] = bd_name;
        }
        fprintf(stdout, "\n*************************************************");
        fprintf(stdout, "\n Pairing state for %s is BONDED", bd_name.c_str());
        fprintf(stdout, "\n*************************************************\n");
        g_bt_app->status.pairing_cmd = COMMAND_COMPLETE;

    } else if (new_state == BT_BOND_STATE_NONE) {
        if (it != bonded_devices.end()) {
            bonded_devices.erase(it);
        }
        fprintf(stdout, "\n*************************************************");
        fprintf(stdout, "\n Pairing state for %s is BOND NONE", bd_name.c_str());
        fprintf(stdout, "\n*************************************************\n");
        g_bt_app->status.pairing_cmd = COMMAND_COMPLETE;
    }
}

void BluetoothApp:: HandleUnPair(bt_bdaddr_t bd_addr ) {
    bdstr_t bd_str;
    std::map<std::string, std::string>::iterator it;
    bdaddr_to_string(&bd_addr, &bd_str[0], sizeof(bd_str));
    std::string deviceAddress(bd_str);

    it = bonded_devices.find(deviceAddress);
    if (it != bonded_devices.end())
        bt_interface->remove_bond(&bd_addr);
    else
        fprintf( stdout, " Device is not in bonded list\n");
}

bt_bdaddr_t BluetoothApp:: AddFoundedDevice(std::string bd_name, bt_bdaddr_t bd_addr ) {

    ALOGI(LOGTAG " Adding Device to inquiry list");
    std::map<std::string, std::string>::iterator it;
    bdstr_t bd_str;
    bdaddr_to_string(&bd_addr, &bd_str[0], sizeof(bd_str));
    std::string deviceAddress(bd_str);

    it = inquiry_list.find(deviceAddress);
    if (it != inquiry_list.end()) {
        return (bd_addr);
    } else {
        inquiry_list[deviceAddress] = bd_name;
        return bd_addr;
    }
}

bt_state_t BluetoothApp:: GetState() {
    return bt_state;
}

void BluetoothApp:: PrintInquiryList() {

    std::cout << "\n**************************** Inquiry List \
*********************************\n";
    std::map<std::string, std::string>::iterator it;
    for (it = inquiry_list.begin(); it != inquiry_list.end(); ++it) {
        std::cout << std::left << std::setw(50) << it->second << std::left <<
        std::setw(50) << it->first << std::endl;
    }
    std::cout << "**************************** End of List \
*********************************\n";
}


void BluetoothApp:: PrintBondedDeviceList() {

    std::cout <<"\n**************************** Bonded Device List \
**************************** \n";
    std::map<std::string, std::string>::iterator it;
    for (it = bonded_devices.begin(); it != bonded_devices.end(); ++it) {
            std::cout << std::left << std::setw(50) << it->second
             << std::left << std::setw(50) << it->first << std::endl;
    }
    std::cout<< "****************************  End of List \
*********************************\n";
}

bool BluetoothApp :: LoadBtStack (void) {
    hw_module_t *module;

    if (hw_get_module (BT_STACK_MODULE_ID, (hw_module_t const **) &module)) {
        ALOGE(LOGTAG " hw_get_module failed");
        return false;
    }

    if (module->methods->open (module, BT_STACK_MODULE_ID, &device_)) {
        return false;
    }

    bt_device_ = (bluetooth_device_t *) device_;
    bt_interface = bt_device_->get_bluetooth_interface ();
    if (!bt_interface) {
        bt_device_->common.close ((hw_device_t *) & bt_device_->common);
        bt_device_ = NULL;
        return false;
    }
    return true;
}


void BluetoothApp :: UnLoadBtStack (void)
{
    if (bt_interface) {
        bt_interface->cleanup ();
        bt_interface = NULL;
    }

    if (bt_device_) {
        bt_device_->common.close ((hw_device_t *) & bt_device_->common);
        bt_device_ = NULL;
    }
}


void BluetoothApp :: InitHandler (void) {

    if (!LoadBtStack())
        return;

    // Starting GAP Thread
    threadInfo[THREAD_ID_GAP].thread_id = thread_new (
            threadInfo[THREAD_ID_GAP].thread_name);

    if (threadInfo[THREAD_ID_GAP].thread_id) {
        g_gap = new Gap (bt_interface, config);
    }

    if ((is_hfp_client_enabled_) || (is_a2dp_sink_enabled_)) {
        // we need to start BT-AM if either of A2DP_SINK or HFP-Client is enabled
        threadInfo[THREAD_ID_BT_AM].thread_id = thread_new (
                        threadInfo[THREAD_ID_BT_AM].thread_name);
        if (threadInfo[THREAD_ID_BT_AM].thread_id) {
             pBTAM = new BT_Audio_Manager (bt_interface, config);
        }
    }

    if(is_a2dp_sink_enabled_) {
        threadInfo[THREAD_ID_A2DP_SINK].thread_id = thread_new (
                threadInfo[THREAD_ID_A2DP_SINK].thread_name);

        if (threadInfo[THREAD_ID_A2DP_SINK].thread_id) {
            pA2dpSink = new A2dp_Sink (bt_interface, config);
        }
    }

    if(is_hfp_client_enabled_) {
        threadInfo[THREAD_ID_HFP_CLIENT].thread_id = thread_new (
                threadInfo[THREAD_ID_HFP_CLIENT].thread_name);

        if (threadInfo[THREAD_ID_HFP_CLIENT].thread_id) {
            pHfpClient = new Hfp_Client(bt_interface, config);
        }
    }

    // registers reactors for socket
    if (is_socket_input_enabled_) {
        if(LocalSocketCreate() != -1) {
            listen_reactor_ = reactor_register (
                thread_get_reactor (threadInfo[THREAD_ID_MAIN].thread_id),
                listen_socket_local_, NULL, BtSocketListenHandler, NULL);
        }
    }

    // Enable Bluetooth
    if (is_bt_enable_default_) {

        BtEvent *event = new BtEvent;
        event->event_id = GAP_API_ENABLE;
        ALOGV (LOGTAG "  Posting enable to GAP thread");
        PostMessage (THREAD_ID_GAP, event);

    }

    if (is_pan_enable_default_) {
        // Starting PAN Thread
        threadInfo[THREAD_ID_PAN].thread_id = thread_new (
            threadInfo[THREAD_ID_PAN].thread_name);

        if (threadInfo[THREAD_ID_PAN].thread_id)
            g_pan = new Pan (bt_interface, config);
    }

    if (is_gatt_enable_default_) {
        ALOGV (LOGTAG "  Starting GATT thread");
        threadInfo[THREAD_ID_GATT].thread_id = thread_new (
            threadInfo[THREAD_ID_GATT].thread_name);

        if (threadInfo[THREAD_ID_GATT].thread_id)
            g_gatt = new Gatt(bt_interface, config);
    }

    // Enable Command line input
    if (is_user_input_enabled_) {
        cmd_reactor_ = reactor_register (thread_get_reactor
                        (threadInfo[THREAD_ID_MAIN].thread_id),
                        STDIN_FILENO, NULL, BtCmdHandler, NULL);
    }
}


void BluetoothApp :: DeInitHandler (void) {
    UnLoadBtStack ();

     // de-register reactors for socket
    if (is_socket_input_enabled_) {
        if(listen_reactor_)
            reactor_unregister ( listen_reactor_);
        if(accept_reactor_)
            reactor_unregister ( accept_reactor_);
    }

    if ((is_hfp_client_enabled_) || (is_a2dp_sink_enabled_)) {
        if (threadInfo[THREAD_ID_BT_AM].thread_id != NULL) {
            thread_free (threadInfo[THREAD_ID_BT_AM].thread_id);
            if ( pBTAM != NULL)
                delete pBTAM;
        }
    }

    if(is_a2dp_sink_enabled_) {
        //STOP A2dp Sink thread
        if (threadInfo[THREAD_ID_A2DP_SINK].thread_id != NULL) {
            thread_free (threadInfo[THREAD_ID_A2DP_SINK].thread_id);
            if ( pA2dpSink != NULL)
                delete pA2dpSink;
        }
    }

    if(is_hfp_client_enabled_) {
        //STOP HFP client thread
        if (threadInfo[THREAD_ID_HFP_CLIENT].thread_id != NULL) {
            thread_free (threadInfo[THREAD_ID_HFP_CLIENT].thread_id);
            if ( pHfpClient != NULL)
                delete pHfpClient;
        }
    }


    // Stop GAP Thread
    if (threadInfo[THREAD_ID_GAP].thread_id != NULL) {
        thread_free (threadInfo[THREAD_ID_GAP].thread_id);
        if ( g_gap != NULL)
            delete g_gap;
    }

    if (is_pan_enable_default_) {
        // Stop PAN Thread
        if (threadInfo[THREAD_ID_PAN].thread_id != NULL) {
            thread_free (threadInfo[THREAD_ID_PAN].thread_id);
            if (g_pan != NULL)
                delete g_pan;
        }
    }
    if (is_gatt_enable_default_) {
        if (threadInfo[THREAD_ID_GATT].thread_id != NULL){
            thread_free(threadInfo[THREAD_ID_GATT].thread_id);
            if (g_gatt != NULL)
                delete g_gatt;
        }
    }

    // Stop Command Handler
    if (is_user_input_enabled_) {
        reactor_unregister (cmd_reactor_);
    }
}

BluetoothApp :: BluetoothApp () {

    // Initial values
    is_bt_enable_default_ = false;
    is_user_input_enabled_ = false;
    ssp_notification = false;
    pin_notification =false;
    listen_socket_local_ = -1;
    client_socket_ = -1;
    cmd_reactor_ = NULL;
    listen_reactor_ = NULL;
    accept_reactor_ = NULL;

    bt_state = BT_STATE_OFF;
    bt_discovery_state = BT_DISCOVERY_STOPPED;

    memset (&status, '\0', sizeof (UiCommandStatus));
    config = NULL;

    if (!LoadConfigParameters (CONFIG_FILE_PATH))
        ALOGE (LOGTAG " Error in Loading config file");
}


BluetoothApp :: ~BluetoothApp () {
    if (config)
        config_free(config);

    bonded_devices.clear();
    inquiry_list.clear();
}

int BluetoothApp:: LocalSocketCreate(void) {
  int conn_sk, length;
  struct sockaddr_un addr;

  listen_socket_local_ = socket(AF_LOCAL, SOCK_STREAM, 0);
  if(listen_socket_local_ < 0) {
    ALOGE (LOGTAG "Failed to create Local Socket 1 (%s)", strerror(errno));
    return -1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_LOCAL;
  strncpy(addr.sun_path, LOCAL_SOCKET_NAME, sizeof(addr.sun_path)-1);
  unlink(LOCAL_SOCKET_NAME);
  if (bind(listen_socket_local_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    ALOGE (LOGTAG "Failed to create Local Socket (%s)", strerror(errno));
    return -1;
  }

  if (listen(listen_socket_local_, 1) < 0) {
    ALOGE (LOGTAG "Local socket listen failed (%s)", strerror(errno));
    close(listen_socket_local_);
    return -1;
  }
  return listen_socket_local_;
}

bool BluetoothApp::LoadConfigParameters (const char *configpath) {

    config = config_new (configpath);
    if (!config) {
        ALOGE (LOGTAG " Unable to open config file");
        return false;
    }

    // checking for the BT Enable option in config file
    is_bt_enable_default_ = config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_ENABLE_DEFAULT, false);

    //checking for user input
    is_user_input_enabled_ = config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_USER_INPUT, false);
    //checking for socket handler
    is_socket_input_enabled_ = config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_SOCKET_ENABLED, false);

    //checking for a2dp sink
    is_a2dp_sink_enabled_ = config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_A2DP_SINK_ENABLED, false);
    //checking for hfp client
    is_hfp_client_enabled_ = config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_HFP_CLIENT_ENABLED, false);
    //checking for Pan handler
    is_pan_enable_default_ = config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_PAN_ENABLED, false);

    //checking for Gatt handler
    is_gatt_enable_default_= config_get_bool (config, CONFIG_DEFAULT_SECTION,
                                    BT_GATT_ENABLED, false);
    return true;
}
