/*  Copyright (C) 2026 pablobc-mx

    This file is part of Gadgetbridge.

    Gadgetbridge is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Gadgetbridge is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>. */
package nodomain.freeyourgadget.gadgetbridge.service.devices.pinetime;

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.os.Handler;
import android.os.Looper;
import android.widget.Toast;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import nodomain.freeyourgadget.gadgetbridge.R;
import nodomain.freeyourgadget.gadgetbridge.devices.pinetime.PineTimeJFConstants;
import nodomain.freeyourgadget.gadgetbridge.entities.PineTimeActivitySample;
import nodomain.freeyourgadget.gadgetbridge.model.ActivityKind;
import nodomain.freeyourgadget.gadgetbridge.service.btle.AbstractBTLEOperation;
import nodomain.freeyourgadget.gadgetbridge.service.btle.TransactionBuilder;
import nodomain.freeyourgadget.gadgetbridge.service.devices.miband.operations.OperationStatus;
import nodomain.freeyourgadget.gadgetbridge.util.GB;

/**
 * Downloads the watch's /SLEEP.BIN file over the InfiniTime Adafruit BLE File Transfer
 * service (0xFEBB) and stores the contained sleep sessions as per-minute LIGHT_SLEEP
 * samples, so Gadgetbridge's sleep charts can display them.
 *
 * <p>Protocol: one command is written to the transfer characteristic, the watch answers
 * with exactly one notification per command. A READ (0x10) opens the file (first response
 * carries the total file length), subsequent chunks are requested with READ_PACING (0x12)
 * which reuses the path of the preceding READ. All fields are little-endian.
 *
 * <p>Each completed sleep session is stored as a 7 byte record: uint32 startTs,
 * uint16 durationMin, uint8 quality. Every minute of a session becomes one
 * PineTimeActivitySample with rawKind = LIGHT_SLEEP, steps = 0 and heartRate = 0;
 * the existing persist path in PineTimeJFSupport merges with any already-stored sample
 * at the same timestamp so step/heart-rate data is not lost.
 */
public class FetchSleepDataOperation extends AbstractBTLEOperation<PineTimeJFSupport> {
    private static final Logger LOG = LoggerFactory.getLogger(FetchSleepDataOperation.class);

    // FS protocol opcodes (FSService.h)
    private static final byte FS_CMD_READ = 0x10;
    private static final byte FS_CMD_READ_DATA = 0x11; // response command used by the firmware
    private static final byte FS_CMD_READ_PACING = 0x12;

    // FS status semantics (deviation from Adafruit spec): 0x01 = success,
    // negative values are LittleFS error codes; LFS_ERR_NOENT (-2, 0xFE) = file not found
    private static final byte FS_STATUS_OK = 0x01;
    private static final byte FS_STATUS_FILE_NOT_FOUND = (byte) 0xFE;

    // READ_DATA response header: cmd(1) status(1) padding(2) chunkoff(4) totallen(4) chunklen(4)
    private static final int FS_READ_RESPONSE_HEADER_SIZE = 16;

    // Sleep record: uint32 startTs, uint16 durationMin, uint8 quality
    private static final int SLEEP_RECORD_SIZE = 7;

    private static final String SLEEP_BIN_PATH = "/SLEEP.BIN";

    // The firmware does not clamp the requested chunk size to the ATT MTU, so the client
    // must request chunksize <= negotiated MTU - 19 (notify payload MTU-3 minus the
    // 16 byte READ_DATA header), otherwise no response ever arrives.
    // PineTimeJF requests MTU 256 at init (negotiated 247), so 228 fits exactly.
    private static final int DEFAULT_CHUNK_SIZE = 228;

    private static final long CHUNK_TIMEOUT_MS = 10_000;
    private static final long TOTAL_TIMEOUT_MS = 60_000;

    private final PineTimeJFSupport support;

    private final Handler watchdogHandler = new Handler(Looper.getMainLooper());
    private final Runnable watchdogRunnable = new Runnable() {
        @Override
        public void run() {
            checkWatchdog();
        }
    };

    private byte[] fileData;
    private int bytesReceived = 0;
    private int totalLen = -1;
    private int nextReadOffset = 0;
    private int chunkSize = DEFAULT_CHUNK_SIZE;
    private boolean transferFinished = false;
    private volatile long lastActivityTimestamp = 0;
    private long transferStartTimestamp = 0;

    public FetchSleepDataOperation(PineTimeJFSupport support) {
        super(support);
        this.support = support;
    }

    private BluetoothGattCharacteristic getTransferCharacteristic() {
        return getCharacteristic(PineTimeJFConstants.UUID_CHARACTERISTIC_FS_TRANSFER);
    }

    private void enableRequiredNotifications(boolean enable) {
        try {
            TransactionBuilder builder = performInitialized("enableRequiredNotifications");
            builder.setCallback(this); // route GATT events to this operation
            builder.notify(getTransferCharacteristic(), enable);
            builder.queue(getQueue());
        } catch (IOException e) {
            LOG.error("Error enabling required notifications", e);
        }
    }

    /** Builds a READ (0x10) request: cmd, padding, pathlen, chunkoff, chunksize, path. */
    private byte[] buildReadCommand(int offset, int chunkSize) {
        byte[] pathBytes = SLEEP_BIN_PATH.getBytes(StandardCharsets.UTF_8);
        ByteBuffer buffer = ByteBuffer.allocate(12 + pathBytes.length).order(ByteOrder.LITTLE_ENDIAN);
        buffer.put(FS_CMD_READ);
        buffer.put((byte) 0x00); // padding
        buffer.putShort((short) pathBytes.length);
        buffer.putInt(offset);
        buffer.putInt(chunkSize);
        buffer.put(pathBytes);
        return buffer.array();
    }

    /** Builds a READ_PACING (0x12) request; reuses the path of the preceding READ. */
    private byte[] buildReadPacingCommand(int offset, int chunkSize) {
        ByteBuffer buffer = ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN);
        buffer.put(FS_CMD_READ_PACING);
        buffer.put((byte) 0x01); // status, ignored by the firmware
        buffer.putShort((short) 0x0000); // padding
        buffer.putInt(offset);
        buffer.putInt(chunkSize);
        return buffer.array();
    }

    private void sendReadCommand() throws IOException {
        TransactionBuilder builder = performInitialized("read sleep data");
        builder.setCallback(this); // route GATT events to this operation
        builder.write(getTransferCharacteristic(), buildReadCommand(nextReadOffset, chunkSize));
        builder.queue(getQueue());
        lastActivityTimestamp = System.currentTimeMillis();
    }

    private void sendReadPacingCommand() throws IOException {
        TransactionBuilder builder = performInitialized("continue reading sleep data");
        builder.setCallback(this); // route GATT events to this operation
        builder.write(getTransferCharacteristic(), buildReadPacingCommand(nextReadOffset, chunkSize));
        builder.queue(getQueue());
        lastActivityTimestamp = System.currentTimeMillis();
    }

    @Override
    protected void prePerform() throws IOException {
        super.prePerform();
        getDevice().setBusyTask("FetchSleepDataOperation starting..."); // mark as busy quickly to avoid interruptions from the outside
        GB.updateTransferNotification(null, getContext().getString(R.string.busy_task_fetch_activity_data), true, 0, getContext());
    }

    @Override
    protected void doPerform() throws IOException {
        if (getTransferCharacteristic() == null) {
            // FS service not exposed by this firmware, nothing we can do
            LOG.warn("FS service not available on this firmware, cannot fetch sleep data");
            operationFinished();
            return;
        }
        enableRequiredNotifications(true);
        startWatchdog();
        sendReadCommand();
    }

    @Override
    protected void operationFinished() {
        LOG.info("FetchSleepDataOperation finished");
        transferFinished = true;
        stopWatchdog();
        unsetBusy();
        GB.updateTransferNotification(null, getContext().getString(R.string.busy_task_fetch_activity_data), false, 100, getContext());
        GB.signalActivityDataFinish(getDevice());

        operationStatus = OperationStatus.FINISHED;
        if (getDevice() != null) {
            try {
                TransactionBuilder builder = performInitialized("finished operation");
                builder.setCallback(null); // unset ourselves from being the queue's gatt callback
                builder.notify(getTransferCharacteristic(), false);
                builder.wait(0);
                builder.queue(getQueue());
            } catch (IOException ex) {
                LOG.error("Error resetting Gatt callback", ex);
            }
        }
    }

    @Override
    public boolean onCharacteristicChanged(BluetoothGatt gatt,
                                           BluetoothGattCharacteristic characteristic) {
        UUID characteristicUUID = characteristic.getUuid();
        byte[] data = characteristic.getValue();

        if (characteristicUUID.equals(PineTimeJFConstants.UUID_CHARACTERISTIC_FS_TRANSFER)) {
            if (transferFinished) {
                return true;
            }
            handleReadResponse(gatt, data);
            return true;
        }
        return super.onCharacteristicChanged(gatt, characteristic);
    }

    private void handleReadResponse(BluetoothGatt gatt, byte[] data) {
        lastActivityTimestamp = System.currentTimeMillis();

        if (data.length < FS_READ_RESPONSE_HEADER_SIZE) {
            LOG.warn("Short READ_DATA response ({} bytes)", data.length);
            return;
        }

        ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        byte command = buffer.get();
        byte status = buffer.get();
        buffer.getShort(); // padding
        int chunkOffset = buffer.getInt();
        int totalLen = buffer.getInt();
        int chunkLen = buffer.getInt();

        if (command != FS_CMD_READ_DATA) {
            LOG.warn("Unexpected FS response command: 0x{:02x}", command);
            return;
        }

        if (status == FS_STATUS_FILE_NOT_FOUND) {
            // No sleep data recorded yet — this is a normal, successful outcome
            LOG.info("No sleep data yet ({} not found), nothing to import", SLEEP_BIN_PATH);
            operationFinished();
            return;
        }

        if (status != FS_STATUS_OK) {
            LOG.error("FS read failed with status 0x{:02x}", status);
            GB.toast(getContext(), "Failed to read sleep data from watch (status " + String.format("0x%02x", status) + ")", Toast.LENGTH_LONG, GB.ERROR);
            operationFinished();
            return;
        }

        if (chunkLen > data.length - FS_READ_RESPONSE_HEADER_SIZE || chunkOffset + chunkLen > totalLen) {
            LOG.error("Invalid chunk in READ_DATA response: offset={} len={} total={} payload={}", chunkOffset, chunkLen, totalLen, data.length);
            operationFinished();
            return;
        }

        this.totalLen = totalLen;
        if (fileData == null || fileData.length != totalLen) {
            fileData = new byte[totalLen];
            bytesReceived = 0;
        }
        System.arraycopy(data, FS_READ_RESPONSE_HEADER_SIZE, fileData, chunkOffset, chunkLen);
        bytesReceived += chunkLen;

        GB.updateTransferNotification(null, getContext().getString(R.string.busy_task_fetch_activity_data), true,
                totalLen > 0 ? Math.min(99, (int) ((long) bytesReceived * 100 / totalLen)) : 99, getContext());

        if (bytesReceived >= totalLen) {
            LOG.info("READ {} total={} bytes", SLEEP_BIN_PATH, totalLen);
            nextReadOffset = 0;
            parseAndPersist(fileData);
            operationFinished();
            return;
        }

        nextReadOffset = chunkOffset + chunkLen;
        try {
            sendReadPacingCommand();
        } catch (IOException e) {
            LOG.error("Error requesting next sleep data chunk", e);
            operationFinished();
        }
    }

    @Override
    public boolean onCharacteristicWrite(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic,
                                         int status) {
        UUID characteristicUUID = characteristic.getUuid();

        if (characteristicUUID.equals(PineTimeJFConstants.UUID_CHARACTERISTIC_FS_TRANSFER)) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                LOG.error("FS write failed with GATT status {}", status);
                if (status == 137) {
                    // BLE_ATT_ERR_INSUFFICIENT_AUTHOR: FS access is disabled on the watch
                    GB.toast(getContext(), "Enable OTA/FS access on the watch (Settings -> OTA/FS access)", Toast.LENGTH_LONG, GB.ERROR);
                } else {
                    GB.toast(getContext(), "Failed to write to watch FS service (GATT status " + status + ")", Toast.LENGTH_LONG, GB.ERROR);
                }
                operationFinished();
            }
            return true;
        }
        return super.onCharacteristicWrite(gatt, characteristic, status);
    }

    private void parseAndPersist(byte[] data) {
        List<SleepRecord> records = parseSleepRecords(data);
        LOG.info("Parsed {} sleep records", records.size());

        if (records.isEmpty()) {
            return;
        }

        List<PineTimeActivitySample> samples = new ArrayList<>();
        for (SleepRecord record : records) {
            for (int i = 0; i < record.durationMin; i++) {
                PineTimeActivitySample sample = new PineTimeActivitySample();
                sample.setTimestamp(record.startTs + i * 60);
                sample.setRawKind(ActivityKind.LIGHT_SLEEP.getCode());
                sample.setSteps(0);
                sample.setHeartRate(0);
                samples.add(sample);
            }
        }

        // The support's persist path acquires the DB, sets device/user/provider and merges
        // with any existing sample at the same timestamp (max steps/heartRate), then
        // insertOrReplace — so no step/heart-rate data is lost on PK collisions.
        support.addGBActivitySamples(samples.toArray(new PineTimeActivitySample[0]));
        LOG.info("Saved {} sleep samples", samples.size());
    }

    /**
     * Parses and validates the /SLEEP.BIN payload. Every completed sleep session is a
     * 7 byte little-endian record: uint32 startTs, uint16 durationMin, uint8 quality.
     * Invalid records are skipped (mirroring the firmware's own guards).
     */
    private List<SleepRecord> parseSleepRecords(byte[] data) {
        List<SleepRecord> records = new ArrayList<>();
        if (data.length == 0) {
            return records;
        }
        if (data.length % SLEEP_RECORD_SIZE != 0) {
            LOG.warn("Invalid {} size {} (not a multiple of {}), ignoring trailing bytes", SLEEP_BIN_PATH, data.length, SLEEP_RECORD_SIZE);
        }

        long now = System.currentTimeMillis() / 1000;
        int validRecords = 0;
        int skippedRecords = 0;
        for (int offset = 0; offset + SLEEP_RECORD_SIZE <= data.length; offset += SLEEP_RECORD_SIZE) {
            ByteBuffer buffer = ByteBuffer.wrap(data, offset, SLEEP_RECORD_SIZE).order(ByteOrder.LITTLE_ENDIAN);
            long startTs = buffer.getInt() & 0xFFFFFFFFL;
            int durationMin = buffer.getShort() & 0xFFFF;
            int quality = buffer.get() & 0xFF;

            if (startTs < 100000 || startTs > now + 60) {
                skippedRecords++;
                continue;
            }
            if (durationMin < 1 || durationMin > 1440) {
                skippedRecords++;
                continue;
            }
            if (quality > 100) {
                skippedRecords++;
                continue;
            }

            records.add(new SleepRecord((int) startTs, durationMin, quality));
            validRecords++;
        }
        if (skippedRecords > 0) {
            LOG.warn("Skipped {} invalid sleep records", skippedRecords);
        }
        LOG.debug("Parsed {} valid sleep records out of {} total", validRecords, records.size() + skippedRecords);
        return records;
    }

    private void startWatchdog() {
        transferStartTimestamp = System.currentTimeMillis();
        lastActivityTimestamp = transferStartTimestamp;
        watchdogHandler.postDelayed(watchdogRunnable, CHUNK_TIMEOUT_MS / 2);
    }

    private void stopWatchdog() {
        watchdogHandler.removeCallbacks(watchdogRunnable);
    }

    private void checkWatchdog() {
        if (transferFinished) {
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastActivityTimestamp > CHUNK_TIMEOUT_MS || now - transferStartTimestamp > TOTAL_TIMEOUT_MS) {
            LOG.error("Sleep data transfer timed out");
            GB.toast(getContext(), "Sleep data transfer timed out", Toast.LENGTH_LONG, GB.ERROR);
            operationFinished();
        } else {
            watchdogHandler.postDelayed(watchdogRunnable, CHUNK_TIMEOUT_MS / 2);
        }
    }

    private static class SleepRecord {
        final int startTs;
        final int durationMin;
        @SuppressWarnings("unused")
        final int quality;

        SleepRecord(int startTs, int durationMin, int quality) {
            this.startTs = startTs;
            this.durationMin = durationMin;
            this.quality = quality;
        }
    }
}
