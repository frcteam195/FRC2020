package com.team195.frc.ckcoprocessor;

import com.google.protobuf.InvalidProtocolBufferException;
import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCPortIn;
import com.team195.frc.constants.Constants;
import com.team195.lib.util.arm.ARMCachedValue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

public class CKCoprocessorDataReceiver {
	private static final CKCoprocessorDataReceiver mInstance = new CKCoprocessorDataReceiver();
	public static CKCoprocessorDataReceiver getInstance() {
		return mInstance;
	}

	private OSCPortIn oscPortIn;
	private List<Object> msgReceivingList = new ArrayList<>();
	private RIOToCoprocessorData.CKRIOData rioToCoprocessorData = RIOToCoprocessorData.CKRIOData.getDefaultInstance();
	private byte[] msgRawData = null;
	private byte[] msgTmpRawData = null;
	private ReentrantLock mDataMutex = new ReentrantLock();

	private ARMCachedValue<RIOToCoprocessorData.CKRIOData> ckrioDataARMCachedValue = new ARMCachedValue<>(5, (t) -> {
		mDataMutex.lock();
		try {
			if (msgRawData != null) {
				msgTmpRawData = new byte[msgRawData.length];
				System.arraycopy(msgRawData, 0, msgTmpRawData, 0, msgRawData.length);
			}
		} catch (Exception ex) {
			msgRawData = null;
			System.out.println("Error transferring bytes from RIO message.");
		} finally {
			mDataMutex.unlock();
		}

		if (msgTmpRawData != null) {
			try {
				rioToCoprocessorData = RIOToCoprocessorData.CKRIOData.parseFrom(msgTmpRawData);
			} catch (InvalidProtocolBufferException ex) {
				rioToCoprocessorData = RIOToCoprocessorData.CKRIOData.getDefaultInstance();
				System.out.println(ex.toString());
			}
		}

		return rioToCoprocessorData;
	});

	private CKCoprocessorDataReceiver() {
		try {
			oscPortIn = new OSCPortIn(Constants.CKCO_TRANSPORT_PORT);
			OSCListener updateListener = (time, message) -> {
				msgReceivingList = message.getArguments();
				if (msgReceivingList != null && msgReceivingList.size() == 1) {
					mDataMutex.lock();
					try {
						msgRawData = (byte[]) msgReceivingList.get(0);
					} catch (Exception ex) {
						msgRawData = null;
						System.out.println("Error parsing data to byte[] from RIO message.");
					} finally {
						mDataMutex.unlock();
					}
				}
			};

			oscPortIn.addListener("/CKCO/CKRIOToCoprocessor", updateListener);
			oscPortIn.startListening();
		} catch (Exception ex) {
			if (oscPortIn != null) {
				try {
					oscPortIn.stopListening();
					oscPortIn.close();
				} catch (Exception ignored) {

				}
			}
			oscPortIn = null;
			System.out.println(ex.toString());
		}
	}

	public RIOToCoprocessorData.CKRIOData getCachedRIOData() {
		return ckrioDataARMCachedValue.getValue();
	}
}
