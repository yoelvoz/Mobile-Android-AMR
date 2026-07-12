package com.example.percobaan1;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.graphics.Color;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import info.mqtt.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import org.json.JSONObject;

import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";
    private MqttAndroidClient mqttClient;

    TextView tvRobot, tvMQTT, tvGPS, tvSpeed, tvDistance, tvWeight, tvBattery, tvCamera;
    Button btnDelivery, btnPickup, btnStandby, btnMapTracking;

    Handler handler = new Handler();

    // ── Data robot ────────────────────────────────────────────────
    String modeLabel         = "STANDBY";
    double robotLat          = 0, robotLon = 0;
    double speed             = 0;
    double distanceRemaining = 0;
    double etaKalman         = 0, etaRaw = 0;
    double weight            = 0;

    double    battery        = 0;
    int currentWP = 0;
    boolean cameraOn = false;

    // ── MQTT config ───────────────────────────────────────────────
    static final String BROKER_URI    = "tcp://172.20.10.4:1883";
    static final String TOP_COMMAND   = "robot/command";
    static final String TOP_TELEMETRY = "robot/telemetry";

    int      retryDelay = 3000;
    Handler  retryHandler = new Handler();
    Runnable retryRunnable;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        tvRobot             = findViewById(R.id.tvRobot);
        tvMQTT              = findViewById(R.id.tvMQTT);
        tvGPS               = findViewById(R.id.tvGPS);
        tvSpeed             = findViewById(R.id.tvSpeed);
        tvDistance          = findViewById(R.id.tvDistance);
        tvWeight            = findViewById(R.id.tvWeight);
        tvCamera            = findViewById(R.id.tvCamera);
        tvBattery           = findViewById(R.id.tvBattery);
        btnDelivery         = findViewById(R.id.btnDelivery);
        btnPickup           = findViewById(R.id.btnPickup);
        btnStandby          = findViewById(R.id.btnStandby);
        btnMapTracking      = findViewById(R.id.btnMapTracking);

        btnDelivery.setOnClickListener(v    -> sendCommand("DELIVERY"));
        btnPickup.setOnClickListener(v      -> sendCommand("PICKUP"));
        btnStandby.setOnClickListener(v     -> sendCommand("STANDBY"));
        btnMapTracking.setOnClickListener(v ->
                startActivity(new Intent(this, MapActivity.class)));

        connectMQTT();
        handler.post(updateRunnable);
    }

    // ── Refresh UI tiap 1 detik ───────────────────────────────────
    private final Runnable updateRunnable = new Runnable() {
        @Override public void run() {
            boolean ok = mqttClient != null && mqttClient.isConnected();
            tvMQTT.setText(ok ? "MQTT : CONNECTED" : "MQTT : DISCONNECTED");
            tvMQTT.setTextColor(ok ? Color.GREEN : Color.RED);

            tvRobot.setText("ROBOT MODE : " + modeLabel);
            switch (modeLabel) {
                case "DELIVERY": tvRobot.setTextColor(Color.GREEN); break;
                case "PICKUP":   tvRobot.setTextColor(Color.CYAN);  break;
                default:         tvRobot.setTextColor(Color.RED);   break;
            }

            if (robotLat != 0 && robotLon != 0)
                tvGPS.setText(String.format("GPS : %.6f , %.6f", robotLat, robotLon));
            else
                tvGPS.setText("GPS : NOT CONNECTED");

            tvSpeed.setText(String.format("Speed : %.2f m/s", speed));
            tvDistance.setText(String.format(
                    "Distance: %.2f m | ETA-EKF: %.0f s | ETA-GPS: %.0f s", distanceRemaining, etaKalman, etaRaw));
            tvWeight.setText(String.format("Weight : %.2f kg", weight));
            tvCamera.setText(cameraOn ? "Camera : ON" : "Camera : OFF");
            tvCamera.setTextColor(cameraOn ? Color.GREEN : Color.RED);
            tvBattery.setText(String.format("Battery : %.0f%%", battery));

            handler.postDelayed(this, 1000);
        }
    };

    // ── Kirim command ─────────────────────────────────────────────
    private void sendCommand(String cmd) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            Toast.makeText(this, "MQTT not connected", Toast.LENGTH_SHORT).show();
            return;
        }

        try {
            JSONObject obj = new JSONObject();
            obj.put("mode", cmd);

            mqttClient.publish(TOP_COMMAND,
                    new MqttMessage(obj.toString().getBytes()));

            Toast.makeText(this, "Command: " + cmd, Toast.LENGTH_SHORT).show();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // ── MQTT menggunakan MqttAndroidClient (non-blocking) ─────────
    private void connectMQTT() {
        String clientId = "AndroidMain_" + UUID.randomUUID().toString().substring(0, 8);
        mqttClient = new MqttAndroidClient(getApplicationContext(), BROKER_URI, clientId);

        mqttClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean reconnect, String serverURI) {
                Log.d(TAG, "MQTT connectComplete");
                retryDelay = 3000;
                try {
                    mqttClient.subscribe(TOP_TELEMETRY, 0);
                    Log.d(TAG, "Subscribe robot/telemetry");
                } catch (Exception e) { e.printStackTrace(); }
            }

            @Override public void connectionLost(Throwable cause) {
                Log.w(TAG, "MQTT lost");
                scheduleRetry();
            }
            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                Log.d(TAG, "RECV [" + topic + "] : " + new String(message.getPayload()));
                if (!TOP_TELEMETRY.equals(topic)) return;
                String pl = new String(message.getPayload());

                // ACK dikirim LANGSUNG di thread MQTT (bukan UI thread):
                try {
                    JSONObject o = new JSONObject(pl);
                    int pktId = o.optInt("pkt_id", 0);
                    if (pktId > 0) {
                        JSONObject ack = new JSONObject();
                        ack.put("id", pktId);
                        ack.put("ts", System.currentTimeMillis());
                        MqttMessage ackMsg = new MqttMessage(ack.toString().getBytes());
                        ackMsg.setQos(1);
                        mqttClient.publish("robot/ack", ackMsg);
                    }
                } catch (Exception e) { e.printStackTrace(); }

                // Update UI di UI thread (terpisah dari ACK):
                runOnUiThread(() -> {
                    try {
                        JSONObject o = new JSONObject(pl);
                        robotLat          = o.optDouble("lat_raw",  robotLat);
                        robotLon          = o.optDouble("lon_raw",  robotLon);
                        speed             = o.optDouble("speed",    speed);
                        distanceRemaining = o.optDouble("distance", distanceRemaining);
                        etaKalman         = o.optDouble("eta_kf",   etaKalman);
                        etaRaw            = o.optDouble("eta_raw",  etaRaw);
                        double rawWeight  = o.optDouble("load", 0);
                        weight            = (rawWeight >= 0 && rawWeight < 500) ? rawWeight : 0;
                        double battVolt   = o.optDouble("battery", 0);
                        if (battVolt > 1.0)
                            battery = Math.max(0, Math.min(100, (battVolt - 12.0) / (14.6 - 12.0) * 100.0));
                        currentWP         = o.optInt("wp", currentWP);
                        modeLabel         = o.optString("delivery_status", "STANDBY");
                        int navMode       = o.optInt("nav_mode", 0);
                        cameraOn          = o.optBoolean("camera_on", navMode == 1 || navMode == 2);
                    } catch (Exception e) { e.printStackTrace(); }
                });
            }  // ← tutup messageArrived

            @Override public void deliveryComplete(IMqttDeliveryToken token) {}
        });  // ← tutup setCallback anonymous class

        doConnect();  // ← tetap di dalam connectMQTT()
    }  // ← tutup connectMQTT()

    private void doConnect() {
        MqttConnectOptions opts = new MqttConnectOptions();
        opts.setAutomaticReconnect(false);
        opts.setCleanSession(true);
        opts.setConnectionTimeout(10);
        opts.setKeepAliveInterval(20);
        try {
            mqttClient.connect(opts, null, new IMqttActionListener() {

                @Override
                public void onSuccess(IMqttToken t) {

                    Log.d(TAG, "MQTT CONNECT SUCCESS");

                    runOnUiThread(() -> {
                        tvMQTT.setText("MQTT : CONNECTED");
                        tvMQTT.setTextColor(Color.GREEN);
                    });
                }

                @Override
                public void onFailure(IMqttToken t, Throwable e) {

                    Log.e(TAG, "========== MQTT CONNECT FAILED ==========");

                    if (e != null) {
                        Log.e(TAG, "Reason: " + e.getMessage(), e);
                    }

                    runOnUiThread(() -> {
                        tvMQTT.setText("MQTT : DISCONNECTED");
                        tvMQTT.setTextColor(Color.RED);
                    });

                    scheduleRetry();
                }
            });

        } catch (Exception e) {
            Log.e(TAG, "Exception saat connect", e);
            scheduleRetry();
        }
    }

    private void scheduleRetry() {
        if (retryRunnable != null) retryHandler.removeCallbacks(retryRunnable);
        retryRunnable = () -> { if (mqttClient != null && !mqttClient.isConnected()) doConnect(); };
        retryHandler.postDelayed(retryRunnable, retryDelay);
        retryDelay = Math.min(retryDelay * 2, 30_000);
    }

    @Override protected void onDestroy() {
        super.onDestroy();
        handler.removeCallbacksAndMessages(null);
        if (retryRunnable != null) retryHandler.removeCallbacksAndMessages(null);
        try { if (mqttClient != null) mqttClient.disconnect(); } catch (Exception ignored) {}
    }
}