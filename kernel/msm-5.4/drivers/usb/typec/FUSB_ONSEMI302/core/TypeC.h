/*******************************************************************************
 * @file     TypeC.h
 * @author   USB PD Firmware Team
 *
 * Copyright 2018 ON Semiconductor. All rights reserved.
 *
 * This software and/or documentation is licensed by ON Semiconductor under
 * limited terms and conditions. The terms and conditions pertaining to the
 * software and/or documentation are available at
 * http://www.onsemi.com/site/pdf/ONSEMI_T&C.pdf
 * ("ON Semiconductor Standard Terms and Conditions of Sale, Section 8 Software").
 *
 * DO NOT USE THIS SOFTWARE AND/OR DOCUMENTATION UNLESS YOU HAVE CAREFULLY
 * READ AND YOU AGREE TO THE LIMITED TERMS AND CONDITIONS. BY USING THIS
 * SOFTWARE AND/OR DOCUMENTATION, YOU AGREE TO THE LIMITED TERMS AND CONDITIONS.
 ******************************************************************************/
#ifndef _FSC_TYPEC_H_
#define	_FSC_TYPEC_H_

#include "platform.h"
#include "Port.h"
#include <linux/notifier.h>
/* Type C Timing Parameters */
#define tAMETimeout     900 * TICK_SCALE_TO_MS /* Alternate Mode Entry Time */
#define tCCDebounce     120 * TICK_SCALE_TO_MS
#define tPDDebounce     15  * TICK_SCALE_TO_MS
#define tDRPTry         90  * TICK_SCALE_TO_MS
#define tDRPTryWait     600 * TICK_SCALE_TO_MS
#define tErrorRecovery  30  * TICK_SCALE_TO_MS /* Delay in Error Recov State */

#define tDeviceToggle   3   * TICK_SCALE_TO_MS /* CC pin measure toggle */
#define tTOG2           30  * TICK_SCALE_TO_MS /* DRP Toggle timing */
#define tIllegalCable   500 * TICK_SCALE_TO_MS /* Reconnect loop reset time */
#define tOrientedDebug  100 * TICK_SCALE_TO_MS /* DebugAcc Orient Delay */
#define tLoopReset      100 * TICK_SCALE_TO_MS
#define tAttachWaitPoll 20  * TICK_SCALE_TO_MS /* Periodic poll in AW.Src */
#define tAttachWaitAdv  20  * TICK_SCALE_TO_MS /* Switch from Default to correct
                                                  advertisement in AW.Src */

/* Attach-Detach loop count - Halt after N loops */
#define MAX_CABLE_LOOP  20

void StateMachineTypeC(Port_t *port);
void StateMachineDisabled(Port_t *port);
void StateMachineErrorRecovery(Port_t *port);
void StateMachineUnattached(Port_t *port);
void StateMachineAttachWaitSink(Port_t *port);
void StateMachineAttachedSink(Port_t *port);
void StateMachineTryWaitSink(Port_t *port);
void StateMachineDebugAccessorySink(Port_t *port);
void StateMachineTrySink(Port_t *port);
void StateMachineAttachWaitSource(Port_t *port);
void StateMachineTryWaitSource(Port_t *port);
void StateMachineUnattachedSource(Port_t *port);    /* AW.Snk -> Unattached */
void StateMachineAttachedSource(Port_t *port);
void StateMachineTrySource(Port_t *port);
void StateMachineDebugAccessorySource(Port_t *port);
void StateMachineAttachWaitAccessory(Port_t *port);
void StateMachineAudioAccessory(Port_t *port);
void StateMachinePoweredAccessory(Port_t *port);
void StateMachineUnsupportedAccessory(Port_t *port);
void SetStateAudioAccessory(Port_t *port);
void SetStateErrorRecovery(Port_t *port);
void SetStateUnattached(Port_t *port);
void SetStateAttachWaitSink(Port_t *port);
void SetStateAttachedSink(Port_t *port);
void SetStateDebugAccessorySink(Port_t *port);
void RoleSwapToAttachedSink(Port_t *port);
void SetStateTryWaitSink(Port_t *port);
void SetStateTrySink(Port_t *port);
void SetStateAttachWaitSource(Port_t *port);
void SetStateAttachedSource(Port_t *port);
void SetStateDebugAccessorySource(Port_t *port);
void RoleSwapToAttachedSource(Port_t *port);
void SetStateTrySource(Port_t *port);
void SetStateTryWaitSource(Port_t *port);
void SetStateUnattachedSource(Port_t *port);
void SetStateAttachWaitAccessory(Port_t *port);
void SetStateUnsupportedAccessory(Port_t *port);
void SetStatePoweredAccessory(Port_t *port);
void SetStateIllegalCable(Port_t *port);
void StateMachineIllegalCable(Port_t *port);

void updateSourceCurrent(Port_t *port);
void updateSourceMDACHigh(Port_t *port);
void updateSourceMDACLow(Port_t *port);

void ToggleMeasure(Port_t *port);
CCTermType DecodeCCTermination(Port_t *port);
CCTermType DecodeCCTerminationSource(Port_t *port);
FSC_BOOL IsCCPinRa(Port_t *port);
CCTermType DecodeCCTerminationSink(Port_t *port);
void UpdateSinkCurrent(Port_t *port, CCTermType term);
FSC_BOOL VbusVSafe0V (Port_t *port);
FSC_BOOL VbusUnder5V(Port_t *port);

FSC_BOOL isVSafe5V(Port_t *port);
FSC_BOOL isVBUSOverVoltage(Port_t *port, FSC_U16 vbus_mv);

void resetDebounceVariables(Port_t *port);
void setDebounceVariables(Port_t *port, CCTermType term);
void setDebounceVariables(Port_t *port, CCTermType term);
void debounceCC(Port_t *port);

void setStateSource(Port_t *port, FSC_BOOL vconn);
void DetectCCPinSource(Port_t *port);
void updateVCONNSource(Port_t *port);
void updateVCONNSource(Port_t *port);
void setStateSink(Port_t *port);
void DetectCCPinSink(Port_t *port);
void updateVCONNSource(Port_t *port);
void updateVCONNSink(Port_t *port);
void clearState(Port_t *port);

void UpdateCurrentAdvert(Port_t *port, USBTypeCCurrent Current);

void SetStateDisabled(Port_t *port);

/* Returns local registers as data array */
FSC_BOOL GetLocalRegisters(Port_t *port, FSC_U8 * data, FSC_S32 size);

void setAlternateModes(FSC_U8 mode);
FSC_U8 getAlternateModes(void);

#endif/* _FSC_TYPEC_H_ */
extern struct atomic_notifier_head audio_switch_notifier;
extern int audio_switch_reg_notifier(struct notifier_block *nb);
extern void audio_switch_unreg_notifier(struct notifier_block *nb);
enum audio_switch_notifier_events {
	PSY_EVENT_PROP_CHANGED_AUDIO,
};
extern int audio_switch_state;