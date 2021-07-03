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


#ifndef BT_APP_HPP
#define BT_APP_HPP

#include "osi/include/thread.h"
#include "osi/include/reactor.h"
#include "osi/include/alarm.h"
#include "osi/include/config.h"
#include "gap/include/Gap.hpp"
#include <hardware/bluetooth.h>
#include "include/ipc.h"
#include "utils.h"

#include <cutils/sockets.h>
#include <sys/un.h>
#include <sys/poll.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/types.h>

/**
 * @file Main.hpp
 * @brief Main header file for the BT application
*/

/**
 * Maximum argument length
 */
#define COMMAND_ARG_SIZE     50

/**
 * Maximum command length
 */
#define COMMAND_SIZE        200

/**
 * Maximum arguments count
 */
#define MAX_ARGUMENTS        20 //TODO


#define BTM_MAX_LOC_BD_NAME_LEN     248

/**
 * Macro used to find the total commands number
 */
#define  NO_OF_COMMANDS(x)  (sizeof(x) / sizeof((x)[0]))

/**
 * The Configuration options
 */
const char *BT_SOCKET_ENABLED      = "BtSockInputEnabled";
const char *BT_ENABLE_DEFAULT      = "BtEnableByDefault";
const char *BT_USER_INPUT          = "UserInteractionNeeded";
const char *BT_A2DP_SINK_ENABLED   = "BtA2dpSinkEnable";
const char *BT_HFP_CLIENT_ENABLED  = "BtHfClientEnable";

/**
 * The Configuration file path
 */
const char *CONFIG_FILE_PATH       = "/etc/bluetooth/bt_app.conf";

/**
 * To track user command status
 */
typedef enum  {
    COMMAND_NONE = 0,
    COMMAND_INPROGRESS,
    COMMAND_COMPLETE,
} CommandStatus;

/**
 * To track user command status
 */
typedef struct {
    CommandStatus enable_cmd;
    CommandStatus enquiry_cmd;
    CommandStatus stop_enquiry_cmd;
    CommandStatus disable_cmd;
    CommandStatus pairing_cmd;
} UiCommandStatus;

/**
 * list of supported commands
 */
typedef enum {
    BT_ENABLE,
    BT_DISABLE,
    START_ENQUIRY,
    CANCEL_ENQUIRY,
    MAIN_EXIT,
    START_PAIR,
    INQUIRY_LIST,
    BONDED_LIST,
    GET_BT_NAME,
    GET_BT_ADDR,
    SET_BT_NAME,
    UNPAIR,
    GET_BT_STATE,
    TEST_MODE,
    GAP_OPTION,
    TEST_ON_OFF,
    A2DP_SINK,
    CONNECT,
    DISCONNECT,
    PLAY,
    PAUSE,
    STOP,
    FASTFORWARD,
    REWIND,
    FORWARD,
    BACKWARD,
    PAN_OPTION,
    CONNECTED_LIST,
    SET_TETHERING,
    RSP_INIT,
    RSP_START,
    HFP_CLIENT,
    CREATE_SCO_CONN,
    DESTROY_SCO_CONN,
    ACCEPT_CALL,
    REJECT_CALL,
    END_CALL,
    HOLD_CALL,
    RELEASE_HELD_CALL,
    RELEASE_ACTIVE_ACCEPT_WAITING_OR_HELD_CALL,
    SWAP_CALLS,
    ADD_HELD_CALL_TO_CONF,
    RELEASE_SPECIFIED_ACTIVE_CALL,
    PRIVATE_CONSULTATION_MODE,
    PUT_INCOMING_CALL_ON_HOLD,
    ACCEPT_HELD_INCOMING_CALL,
    REJECT_HELD_INCOMING_CALL,
    DIAL,
    REDIAL,
    DIAL_MEMORY,
    START_VR,
    STOP_VR,
    CALL_ACTION,
    QUERY_CURRENT_CALLS,
    QUERY_OPERATOR_NAME,
    QUERY_SUBSCRIBER_INFO,
    SCO_VOL_CTRL,
    MIC_VOL_CTRL,
    SPK_VOL_CTRL,
    SEND_DTMF,
    DISABLE_NREC_ON_AG,
    SEND_AT_CMD,
    BACK_TO_MAIN,
    END,
} CommandList;

/**
 * Argument Count
 */
typedef enum {
    ZERO_PARAM,
    ONE_PARAM,
    TWO_PARAM,
} MaxParamCount;

typedef enum {
    MAIN_MENU,
    GAP_MENU,
    TEST_MENU,
    A2DP_SINK_MENU,
    HFP_CLIENT_MENU,
    PAN_MENU
} MenuType;

/**
 * Default menu_type is Main Menu
 */
MenuType menu_type = MAIN_MENU;

/**
 * UserMenuList
 */
typedef struct {
    CommandList cmd_id;
    const char cmd_name[COMMAND_SIZE];
    MaxParamCount max_param;
    const char cmd_help[COMMAND_SIZE];
} UserMenuList;

/**
 * list of supported commands for GAP
 */
UserMenuList GapMenu[] = {
    {BT_ENABLE,             "enable",           ZERO_PARAM,    "enable"},
    {BT_DISABLE,            "disable",          ZERO_PARAM,    "disable"},
    {START_ENQUIRY,         "inquiry",          ZERO_PARAM,    "inquiry"},
    {CANCEL_ENQUIRY,        "cancel_inquiry",   ZERO_PARAM,    "cancel_inquiry"},
    {START_PAIR,            "pair",             ONE_PARAM,    "pair<space><bt_address> \
    eg. pair 00:11:22:33:44:55"},
    {UNPAIR,                "unpair",           ONE_PARAM,    "unpair<space><bt_address> \
    eg. unpair 00:11:22:33:44:55"},
    {INQUIRY_LIST,          "inquiry_list",     ZERO_PARAM,    "inquiry_list"},
    {BONDED_LIST,           "bonded_list",      ZERO_PARAM,    "bonded_list"},
    {GET_BT_STATE,          "get_state",        ZERO_PARAM,    "get_state"},
    {GET_BT_NAME,           "get_bt_name",      ZERO_PARAM,    "get_bt_name"},
    {GET_BT_ADDR,           "get_bt_address",   ZERO_PARAM,    "get_bt_address"},
    {SET_BT_NAME,           "set_bt_name",      ONE_PARAM,    "set_bt_name<space><bt name> \
    eg. set_bt_name MDM_Fluoride"},
    {BACK_TO_MAIN,          "main_menu",        ZERO_PARAM,    "main_menu"},
};

/**
 * list of supported commands for Main Menu
 */
UserMenuList MainMenu[] = {
    {GAP_OPTION,            "gap_menu",         ZERO_PARAM,   "gap_menu"},
    {PAN_OPTION,            "pan_menu",         ZERO_PARAM,   "pan_menu"},
    {TEST_MODE,             "test_menu",        ZERO_PARAM,   "test_menu"},
    {A2DP_SINK,             "a2dp_sink_menu",   ZERO_PARAM,   "a2dp_sink_menu"},
    {HFP_CLIENT,            "hfp_client_menu",  ZERO_PARAM,   "hfp_client_menu"},
    {MAIN_EXIT,             "exit",             ZERO_PARAM,   "exit"},
};

/**
 * list of supported commands for PAN
 */
UserMenuList PanMenu[] = {
    {SET_TETHERING,  "enable_tethering",         ONE_PARAM, \
    "enable_tethering<space><true or false> eg. enable_tethering true"},
    {DISCONNECT,     "disconnect",               ONE_PARAM, \
    "disconnect<space><bt_address> eg. disconnect 00:11:22:33:44:55"},
    {CONNECTED_LIST, "connected_device_list",    ZERO_PARAM, "connected_device_list"},
    {BACK_TO_MAIN,   "main_menu",                ZERO_PARAM, "main_menu"},
};
/**
 * list of supported commands for Test Menu
 */
UserMenuList TestMenu[] = {
    {TEST_ON_OFF,           "on_off",    ONE_PARAM,     "<on_off> <number>   eg: on_off 100"},
    {BACK_TO_MAIN,          "main_menu", ZERO_PARAM,    "main_menu"},
    {RSP_INIT,              "rsp_init",  ZERO_PARAM,    "rsp_init (only for Init time)"},
    {RSP_START,             "rsp_start", ZERO_PARAM,    "rsp_start would (re)start adv"},
};

/**
 * list of supported commands for A2DP_SINK Menu
 */
UserMenuList A2dpSinkMenu[] = {
    {CONNECT,               "connect",          ONE_PARAM,    "connect<space><bt_address>"},
    {DISCONNECT,            "disconnect",       ONE_PARAM,    "disconnect<space><bt_address>"},
    {PLAY,                  "play",             ZERO_PARAM,    "play"},
    {PAUSE,                 "pause",            ZERO_PARAM,    "pause"},
    {STOP,                  "stop",             ZERO_PARAM,    "stop<"},
    {REWIND,                "rewind",           ZERO_PARAM,    "rewind"},
    {FASTFORWARD,           "fastforward",      ZERO_PARAM,    "fastforward"},
    {FORWARD,               "forward",          ZERO_PARAM,    "forward"},
    {BACKWARD,              "backward",         ZERO_PARAM,    "backward"},
    {BACK_TO_MAIN,          "main_menu",        ZERO_PARAM,    "main_menu"},
};

/**
 * list of supported commands for HFP_CLIENT Menu
 */
UserMenuList HfpClientMenu[] = {
    {CONNECT,               "connect",       ONE_PARAM,    "connect<space><bt_address>"},
    {DISCONNECT,            "disconnect",    ONE_PARAM,    "disconnect<space><bt_address>"},
    {CREATE_SCO_CONN,       "create_sco",    ONE_PARAM,    "create_sco<space><bt_address>"},
    {DESTROY_SCO_CONN,      "destroy_sco",   ONE_PARAM,    "destroy_sco<space><bt_address>"},
    {ACCEPT_CALL,           "accept_call",   ZERO_PARAM,   "accept_call"},
    {REJECT_CALL,           "reject_call",   ZERO_PARAM,   "reject_call"},
    {END_CALL,              "end_call",      ZERO_PARAM,   "end_call"},
    {HOLD_CALL,             "hold_call",     ZERO_PARAM,   "hold_call"},
    {RELEASE_HELD_CALL,     "release_held_call", ZERO_PARAM,   "release_held_call"},
    {RELEASE_ACTIVE_ACCEPT_WAITING_OR_HELD_CALL,  "release_active_accept_waiting_or_held_call",
      ZERO_PARAM,"release_active_accept_waiting_or_held_call"},
    {SWAP_CALLS,            "swap_calls", ZERO_PARAM,   "swap_calls"},
    {ADD_HELD_CALL_TO_CONF, "add_held_call_to_conference", ZERO_PARAM,   "add_held_call_to_conference"},
    {RELEASE_SPECIFIED_ACTIVE_CALL, "release_specified_active_call", ONE_PARAM,
      "release_specified_active_call<space><index of the call>"},
    {PRIVATE_CONSULTATION_MODE, "private_consultation_mode", ONE_PARAM,
      "private_consultation_mode<space><index of the call>"},
    {PUT_INCOMING_CALL_ON_HOLD, "put_incoming_call_on_hold", ZERO_PARAM,   "put_incoming_call_on_hold"},
    {ACCEPT_HELD_INCOMING_CALL, "accept_held_incoming_call", ZERO_PARAM,   "accept_held_incoming_call"},
    {REJECT_HELD_INCOMING_CALL, "reject_held_incoming_call", ZERO_PARAM,   "reject_held_incoming_call"},
    {DIAL,                  "dial",          ONE_PARAM,    "dial<space><phone_number>"},
    {REDIAL,                "redial",        ZERO_PARAM,   "redial"},
    {DIAL_MEMORY,           "dial_memory",   ONE_PARAM,    "dial_memory<space><memory_location>"},
    {START_VR,              "start_vr",      ZERO_PARAM,   "start_vr"},
    {STOP_VR,               "stop_vr",       ZERO_PARAM,   "stop_vr"},
    {QUERY_CURRENT_CALLS,   "query_current_calls", ZERO_PARAM,   "query_current_calls"},
    {QUERY_OPERATOR_NAME,   "query_operator_name", ZERO_PARAM,   "query_operator_name"},
    {QUERY_SUBSCRIBER_INFO, "query_subscriber_info", ZERO_PARAM, "query_subscriber_info"},
    {MIC_VOL_CTRL,          "mic_volume_control",   ONE_PARAM,   "mic_volume_control<space><value>"},
    {SPK_VOL_CTRL,          "speaker_volume_control",   ONE_PARAM,   "speaker_volume_control<space><value>"},
    {SEND_DTMF,             "send_dtmf",   ONE_PARAM,    "send_dtmf<space><code>"},
    {DISABLE_NREC_ON_AG,    "disable_nrec_on_AG",       ZERO_PARAM,   "disable_nrec_on_AG"},
    {BACK_TO_MAIN,          "main_menu",     ZERO_PARAM,   "main_menu"},
};

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief DisplayMenu
 *
 *  It will display list of supported commands based an argument @ref MenuType
 *
 * @param[in] menu_type @ref MenuType specify which menu commands need to display
 * @return none
 */
static void DisplayMenu(MenuType menu_type);

/**
 * @brief HandleUserInput
 *
 *  It will parse user input.
 *
 * @param[out]  int :    cmd_id has command id from @ref CommandList
 * @param[out]  char[][] : input_args contains the command and arguments
 * @param[in]  MenuType : It refers to Menu type currently user in
 */
static bool HandleUserInput (int *cmd_id, char input_args[][COMMAND_ARG_SIZE],
                                                          MenuType menu_type);
/**
 * @brief SignalHandler
 *
 *  It will handle SIGINT signal.
 *
 * @param[in]  int signal number
 * @return none
 */
static void SignalHandler(int sig);

/**
 * @brief ExitHandler
 *
 * This function can be called from main thead or signal handler thread to exit from
 * bt-app
 *
 */
static void ExitHandler(void);

/**
 * @brief HandleMainCommand
 *
 * This will handle all the commands in @ref MainMenu
 *
 * @param[in]  int cmd_id has command id from @ref CommandList
 * @param[in]  char[][] user_cmd has parsed command with arguments passed by user
 * @return none
 */
static void HandleMainCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]);

/**
 * @brief HandleTestCommand
 *
 *  This function will handle all the commands in @ref TestMenu
 *
 * @param[in] cmd_id It has command id from @ref CommandList
 * @param[in] user_cmd It has parsed commands with arguments passed by user
 * @return none
 */
static void HandleTestCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]);

/**
 * @brief HandleGapCommand
 *
 *  This function will handle all the commands in @ref GapMenu
 *
 * @param[in] cmd_id It has command id from @ref CommandList
 * @param[out] user_cmd It has parsed commands with arguments passed by user
 * @return none
 */
static void HandleGapCommand(int cmd_id, char user_cmd[][COMMAND_ARG_SIZE]);

/**
 * @brief BtCmdHandler
 *
 *  This function will take the command as input from the user. And process as per
 *  command recieved
 *
 * @param[in] context
 * @return none
 */
static void BtCmdHandler (void *context);


/**
 * @brief BtCmdHandler
 *
 *  This function will accept the events from other threads and performs action
 *  based on event
 *
 * @param[in] context
 * @return none
 */
void BtMainMsgHandler (void *context);

#ifdef __cplusplus
}
#endif

/**
 * @class BluetoothApp
 *
 * @brief This module will take inputs from command line and also from
 * socket interface. Perform action based on inputs.
 *
 */

class BluetoothApp {
  private:
    config_t *config;
    bool is_bt_enable_default_;
    bool is_user_input_enabled_;
    bool is_socket_input_enabled_;
    bool is_a2dp_sink_enabled_;
    bool is_hfp_client_enabled_;
    bool is_pan_enable_default_;
    bool is_gatt_enable_default_;
    reactor_object_t *cmd_reactor_;
    struct hw_device_t *device_;
    bluetooth_device_t *bt_device_;
    bool LoadConfigParameters(const char *config_path);
    void InitHandler();
    void DeInitHandler();
    bool LoadBtStack();
    void UnLoadBtStack();
    int LocalSocketCreate(void);

  public:
    int listen_socket_local_;
    int client_socket_;
    bool ssp_notification;
    bool pin_notification;

    /**
     * structure object for standard Bluetooth DM interface
     */
    const bt_interface_t *bt_interface;

    reactor_object_t *listen_reactor_;
    reactor_object_t *accept_reactor_;
    UiCommandStatus status;
    bt_state_t bt_state;
    bt_discovery_state_t bt_discovery_state;

    //key is bt_bdaddr_t storing as a string in map
    std::map < std::string, std::string> bonded_devices;
    std::map <std::string, std::string> inquiry_list;
    SSPReplyEvent   ssp_data;
    PINReplyEvent   pin_reply;

    /**
     * @brief AddFoundedDevice
     *
     *This function will stores the device discovered to @ref inquiry_list
     *
     * @param none
     * @return none
     */
    bt_bdaddr_t AddFoundedDevice(std::string bdName, const bt_bdaddr_t bd_addr);
    /**
     * @brief
     * This function will display inquiry list
     *
     * @param none
     * @return none
     */
    void PrintInquiryList();
    /**
     * @brief
     * This function will display Bonded Device list
     *
     * @param none
     * @return none
     */
    void PrintBondedDeviceList();

    /**
     * @brief
     * This function will display Bonded Device list
     *
     * @param none
     * @return none
     */
    void HandleBondState(bt_bond_state_t new_state, const bt_bdaddr_t bd_addr,
                                                    std::string bd_name);

    /**
     * @brief
     *
     * This function will call remove_bond api of stack and removes device from
     * Bonded Device list
     *
     * @param none
     * @return none
     */
    void HandleUnPair(bt_bdaddr_t bd_addr);

    /**
     * @brief Bluetooth Application Constructor
     *
     * It will initialize class members to default values and calls the
     * @ref LoadConfigParameters, this function reads config file @ref CONFIG_FILE_PATH
     */
    BluetoothApp();

    /**
     * @ref Bluetooth Application Distructor
     *
     * It will free the config parameter
     *
     */

    ~BluetoothApp();
    /**
     * @brief GetState
     *
     *  This function will returns the current BT state
     *
     * @return bt_state_t
     */
    bt_state_t GetState();
    /**
     * @brief HandleSspInput
     *
     *  This function will handles the SSP Input, it shows a message on console
     * for user input
     *
     * @return bool
     */
    bool HandleSspInput(char user_cmd[][COMMAND_ARG_SIZE]);
    /**
     * @brief HandlePinInput
     *
     *  This function will take the PIN from user
     *
     * @param[in] user_cmd
     * @return bool
     */
    bool HandlePinInput(char user_cmd[][COMMAND_ARG_SIZE]);

    /**
     *@brief ProcessEvent
     *
     * It will handle incomming events
     *
     * @param BtEvent
     * @return none
     */
    void ProcessEvent(BtEvent * pEvent);
};

#endif
