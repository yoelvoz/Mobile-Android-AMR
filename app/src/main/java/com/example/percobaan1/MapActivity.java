package com.example.percobaan1;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.drawable.BitmapDrawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.SeekBar;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import info.mqtt.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import org.json.JSONArray;
import org.json.JSONObject;

import org.osmdroid.config.Configuration;
import org.osmdroid.events.MapEventsReceiver;
import org.osmdroid.tileprovider.tilesource.TileSourceFactory;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.MapEventsOverlay;
import org.osmdroid.views.overlay.Marker;
import org.osmdroid.views.overlay.Polyline;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MapActivity extends AppCompatActivity {

    // ── VIEWS ──────────────────────────────────────────────────────────────
    MapView     map;
    Button      btnBack, btnStart, btnReset, btnAddWaypoint;
    Button      btnSetPickup, btnSetDestination; // ← BARU
    ImageButton btnCenterRobot;
    TextView    tvStatus, tvDestination, tvSpeed,
            tvRemaining, tvTravelled, tvWeight,
            tvEtaKalman,tvEtaGps,tvWpType;
    TextView    tvPickupInfo, tvDestInfo;
    SeekBar seekSpeed;
    TextView tvSpeedVal;
    float currentSpeed = 0.90f;


    // ── MAP OVERLAY ────────────────────────────────────────────────────────
    Marker         robotMarker;
    Marker         pickupMarker;    // ← BARU
    Marker         destMarker;      // ← BARU
    List<Marker>   waypointMarkers = new ArrayList<>();
    List<GeoPoint> waypoints       = new ArrayList<>();
    Polyline       plannedRoute;
    Polyline       pathKalman;
    List<GeoPoint> pointsKalman = new ArrayList<>();

    // ── STATE ──────────────────────────────────────────────────────────────
    double  robotLat         = 0;
    double  robotLon         = 0;
    double  lastLat          = 0;
    double  lastLon          = 0;
    double osrmDistanceM   = 0;
    double osrmDurationSec = 0;
    boolean isNavigating     = false;
    boolean firstFixReceived = false;
    boolean autoFollow       = true;


    // ── PICKUP / DESTINATION STATE ─────────────────────────────────────────
    GeoPoint pickupPoint    = null;   // ← BARU titik ambil barang
    String   pickupName     = "";
    GeoPoint destPoint      = null;   // ← BARU titik tujuan antar
    String   destName       = "";
    boolean  pickingPickup  = false;  // ← BARU mode klik peta untuk pickup
    boolean  pickingDest    = false;  // ← BARU mode klik peta untuk tujuan

    // ── EXECUTOR untuk HTTP (Nominatim search) ──────────────────────────────
    ExecutorService executor = Executors.newSingleThreadExecutor();

    // ── MQTT ───────────────────────────────────────────────────────────────
    static final String BROKER_URI     = "tcp://172.20.10.4:1883";
    static final String T_TELEMETRY    = "robot/telemetry";
    static final String T_GPS_FILTERED = "robot/gps/filtered";
    static final String T_WAYPOINT_SET = "robot/waypoint/set";
    static final String T_CMD          = "robot/command";

    MqttAndroidClient mqttClient;
    boolean           mqttReady  = false;
    int               retryDelay = 3000;
    Handler           retryHandler    = new Handler(Looper.getMainLooper());
    Runnable          retryRunnable;
    Handler           watchdogHandler  = new Handler(Looper.getMainLooper());
    Runnable          watchdogRunnable;
    static final long WATCHDOG_TIMEOUT = 15_000L;

    // ======================================================================
    //  onCreate
    // ======================================================================
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Configuration.getInstance().load(
                getApplicationContext(),
                PreferenceManager.getDefaultSharedPreferences(getApplicationContext())
        );
        Configuration.getInstance().setUserAgentValue(getPackageName());

        setContentView(R.layout.activity_map);

        bindViews();
        setupMap();
//        addCursorView();
        setupButtons();
        connectMQTT();
    }

    // ======================================================================
    //  BIND VIEWS
    // ======================================================================
    private void bindViews() {
        map               = findViewById(R.id.map);
        btnBack           = findViewById(R.id.btnBack);
        btnStart          = findViewById(R.id.btnStart);
        btnReset          = findViewById(R.id.btnReset);
        btnAddWaypoint    = findViewById(R.id.btnAddWaypoint);
        btnSetPickup      = findViewById(R.id.btnSetPickup);
        btnSetDestination = findViewById(R.id.btnSetDestination);
        btnCenterRobot    = findViewById(R.id.btnCenterRobot);
        tvStatus          = findViewById(R.id.tvStatus);
        tvDestination     = findViewById(R.id.tvDestination);
        tvSpeed           = findViewById(R.id.tvSpeed);
        tvRemaining       = findViewById(R.id.tvRemaining);
        tvWeight          = findViewById(R.id.tvWeight);
        tvEtaKalman       = findViewById(R.id.tvEtaKalman);
        tvEtaGps          = findViewById(R.id.tvEtaGps);
        tvWpType          = findViewById(R.id.tvWpType);
        tvPickupInfo      = findViewById(R.id.tvPickupInfo);
        tvDestInfo        = findViewById(R.id.tvDestInfo);
        seekSpeed  = findViewById(R.id.seekSpeed);
        tvSpeedVal = findViewById(R.id.tvSpeedVal);
    }

    // ======================================================================
    //  SETUP MAP
    // ======================================================================
    private void setupMap() {
        map.setTileSource(TileSourceFactory.MAPNIK);
        map.setMultiTouchControls(true);
        map.setBuiltInZoomControls(false);
        map.getController().setZoom(18.0);

        GeoPoint defaultPos = new GeoPoint(-7.050829, 110.391141);
        map.getController().setCenter(defaultPos);

        // matikan auto-follow saat user scroll manual
        map.addMapListener(new org.osmdroid.events.MapListener() {
            @Override public boolean onScroll(org.osmdroid.events.ScrollEvent e) {
                autoFollow = false; return false;
            }
            @Override public boolean onZoom(org.osmdroid.events.ZoomEvent e) { return false; }
        });

        // ── Listener klik peta untuk mode picking pickup / tujuan ── ← BARU
        MapEventsOverlay mapEventsOverlay = new MapEventsOverlay(new MapEventsReceiver() {
            @Override
            public boolean singleTapConfirmedHelper(GeoPoint p) {
                if (pickingPickup) {
                    pickingPickup = false;
                    setPickupPoint(p, "Map Point");
                    btnSetPickup.setText("📦 Set Pickup");
                    return true;
                }
                if (pickingDest) {
                    pickingDest = false;
                    setDestPoint(p, "Map Point");
                    btnSetDestination.setText("⭐ Set Destination");
                    return true;
                }
                return false;
            }
            @Override public boolean longPressHelper(GeoPoint p) { return false; }
        });
        map.getOverlays().add(0, mapEventsOverlay);

        // planned route biru
        plannedRoute = new Polyline();
        plannedRoute.setWidth(8f);
        plannedRoute.setColor(Color.parseColor("#2962FF"));
        map.getOverlays().add(plannedRoute);

        // trail EKF cyan
        pathKalman = new Polyline();
        pathKalman.setWidth(6f);
        pathKalman.setColor(Color.parseColor("#00D4FF"));
        map.getOverlays().add(pathKalman);

        // robot marker custom
        robotMarker = new Marker(map);
        robotMarker.setPosition(defaultPos);
        robotMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER);
        robotMarker.setTitle("Robot ARGO (EKF)");
        robotMarker.setIcon(new BitmapDrawable(getResources(), makeRobotBitmap()));
        map.getOverlays().add(robotMarker);

        map.invalidate();
    }

    // ======================================================================
    //  CURSOR HIJAU GOJEK STYLE
    // ======================================================================
//    private void addCursorView() {
//        FrameLayout root = findViewById(android.R.id.content);
//
//        View cursor = new View(this) {
//            @Override
//            protected void onDraw(Canvas canvas) {
//                float cx = getWidth()  / 2f;
//                float cy = getHeight() / 2f;
//                float r  = Math.min(cx, cy);
//
//                Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
//
//                p.setStyle(Paint.Style.FILL);
//                p.setColor(Color.parseColor("#4400C853"));
//                canvas.drawCircle(cx, cy, r, p);
//
//                p.setStyle(Paint.Style.STROKE);
//                p.setColor(Color.parseColor("#00C853"));
//                p.setStrokeWidth(dpToPx(2.5f));
//                canvas.drawCircle(cx, cy, r * 0.85f, p);
//
//                p.setStyle(Paint.Style.FILL);
//                p.setColor(Color.WHITE);
//                canvas.drawCircle(cx, cy, r * 0.32f, p);
//
//                p.setColor(Color.parseColor("#00C853"));
//                canvas.drawCircle(cx, cy, r * 0.20f, p);
//            }
//        };
//
//        int size = dpToPx(48);
//        FrameLayout.LayoutParams lp = new FrameLayout.LayoutParams(size, size);
//        lp.gravity = Gravity.CENTER;
//        cursor.setElevation(10f);
//        root.addView(cursor, lp);
//    }

    // ======================================================================
    //  ROBOT BITMAP — lingkaran biru + label ROBOT
    // ======================================================================
    private Bitmap makeRobotBitmap() {
        int size = dpToPx(72);
        Bitmap bmp = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
        Canvas c   = new Canvas(bmp);
        Paint  p   = new Paint(Paint.ANTI_ALIAS_FLAG);

        p.setColor(Color.parseColor("#4400D4FF")); p.setStyle(Paint.Style.FILL);
        c.drawCircle(size/2f, size/2f, size/2f, p);

        p.setColor(Color.parseColor("#1044BB"));
        c.drawCircle(size/2f, size/2f, size*0.36f, p);

        p.setColor(Color.parseColor("#55AAFF")); p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(dpToPx(2));
        c.drawCircle(size/2f, size/2f, size*0.36f, p);

        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.WHITE);
        p.setTextSize(dpToPx(9));
        p.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        p.setTextAlign(Paint.Align.CENTER);
        c.drawText("ROBOT", size/2f, size/2f - (p.descent()+p.ascent())/2f, p);

        return bmp;
    }

    // ======================================================================
    //  WAYPOINT BITMAP — pin hijau + nomor
    // ======================================================================
    private Bitmap makeWaypointBitmap(int number) {
        int w = dpToPx(56), h = dpToPx(72);
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas c   = new Canvas(bmp);
        int cx = w/2, cr = dpToPx(20), headY = cr + dpToPx(4);
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);

        p.setColor(Color.parseColor("#1A8A38")); p.setStyle(Paint.Style.FILL);
        android.graphics.Path tail = new android.graphics.Path();
        tail.moveTo(cx - dpToPx(7), headY + dpToPx(6));
        tail.lineTo(cx,             headY + cr + dpToPx(18));
        tail.lineTo(cx + dpToPx(7), headY + dpToPx(6));
        tail.close();
        c.drawPath(tail, p);

        p.setColor(Color.parseColor("#22AA44"));
        c.drawCircle(cx, headY, cr, p);

        p.setColor(Color.WHITE); p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(dpToPx(2));
        c.drawCircle(cx, headY, cr, p);

        p.setStyle(Paint.Style.FILL); p.setColor(Color.WHITE);
        p.setTextSize(dpToPx(15));
        p.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        p.setTextAlign(Paint.Align.CENTER);
        c.drawText(String.valueOf(number), cx, headY - (p.descent()+p.ascent())/2f, p);

        return bmp;
    }

    // ── BARU: Pickup marker bitmap — pin hijau terang + ikon kotak ─────────
    private Bitmap makePickupBitmap() {
        int w = dpToPx(56), h = dpToPx(72);
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas c   = new Canvas(bmp);
        int cx = w/2, cr = dpToPx(20), headY = cr + dpToPx(4);
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);

        // ekor
        p.setColor(Color.parseColor("#007A30")); p.setStyle(Paint.Style.FILL);
        android.graphics.Path tail = new android.graphics.Path();
        tail.moveTo(cx - dpToPx(7), headY + dpToPx(6));
        tail.lineTo(cx,             headY + cr + dpToPx(18));
        tail.lineTo(cx + dpToPx(7), headY + dpToPx(6));
        tail.close();
        c.drawPath(tail, p);

        // kepala hijau terang
        p.setColor(Color.parseColor("#00D97E"));
        c.drawCircle(cx, headY, cr, p);

        // border putih
        p.setColor(Color.WHITE); p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(dpToPx(2));
        c.drawCircle(cx, headY, cr, p);

        // teks "P"
        p.setStyle(Paint.Style.FILL); p.setColor(Color.parseColor("#003319"));
        p.setTextSize(dpToPx(16));
        p.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        p.setTextAlign(Paint.Align.CENTER);
        c.drawText("P", cx, headY - (p.descent()+p.ascent())/2f, p);

        return bmp;
    }

    // ── BARU: Destination marker bitmap — pin biru + label "D" ─────────────
    private Bitmap makeDestBitmap() {
        int w = dpToPx(56), h = dpToPx(72);
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas c   = new Canvas(bmp);
        int cx = w/2, cr = dpToPx(20), headY = cr + dpToPx(4);
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);

        // ekor
        p.setColor(Color.parseColor("#003B99")); p.setStyle(Paint.Style.FILL);
        android.graphics.Path tail = new android.graphics.Path();
        tail.moveTo(cx - dpToPx(7), headY + dpToPx(6));
        tail.lineTo(cx,             headY + cr + dpToPx(18));
        tail.lineTo(cx + dpToPx(7), headY + dpToPx(6));
        tail.close();
        c.drawPath(tail, p);

        // kepala biru
        p.setColor(Color.parseColor("#4A9EFF"));
        c.drawCircle(cx, headY, cr, p);

        p.setColor(Color.WHITE); p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(dpToPx(2));
        c.drawCircle(cx, headY, cr, p);

        // teks "D"
        p.setStyle(Paint.Style.FILL); p.setColor(Color.WHITE);
        p.setTextSize(dpToPx(16));
        p.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        p.setTextAlign(Paint.Align.CENTER);
        c.drawText("D", cx, headY - (p.descent()+p.ascent())/2f, p);

        return bmp;
    }

    private int dpToPx(float dp) {
        return Math.round(dp * getResources().getDisplayMetrics().density);
    }

    // ======================================================================
    //  SETUP BUTTONS
    // ======================================================================
    private void setupButtons() {
        btnBack.setOnClickListener(v -> finish());
        btnReset.setOnClickListener(v -> resetAll());

        btnCenterRobot.setOnClickListener(v -> {
            if (robotLat != 0 && robotLon != 0) {
                autoFollow = true;
                map.getController().animateTo(new GeoPoint(robotLat, robotLon));
            } else {
                Toast.makeText(this, "GPS not available", Toast.LENGTH_SHORT).show();
            }
        });

        btnAddWaypoint.setOnClickListener(v -> addWaypointAtCenter());

        btnStart.setOnClickListener(v -> {
            if (waypoints.isEmpty()) {
                Toast.makeText(this, "Add a waypoint first!", Toast.LENGTH_SHORT).show();
                return;
            }
            if (!mqttReady) {
                Toast.makeText(this, "MQTT not connected!", Toast.LENGTH_SHORT).show();
                return;
            }
            sendWaypointToRobot();
        });

        // ── BARU: Tombol Pickup ─────────────────────────────────────────────
        btnSetPickup.setOnClickListener(v -> showLocationSearchDialog(true));
        btnSetDestination.setOnClickListener(v -> showLocationSearchDialog(false));

        seekSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar sb, int p, boolean u) {
                currentSpeed = Math.round(p * 5 + 30) / 100f;
                tvSpeedVal.setText(String.format("%.2f m/s", currentSpeed));
                updateEtaDisplay();
            }
            @Override public void onStartTrackingTouch(SeekBar sb) {}
            @Override public void onStopTrackingTouch(SeekBar sb) {
                sendSpeed(currentSpeed);
            }
        });

    }

    // ======================================================================
    //  Menggunakan Nominatim OpenStreetMap API — gratis, tanpa API key
    // ======================================================================
    private void showLocationSearchDialog(boolean isPickup) {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(isPickup ? "📦 Pickup Point" : "⭐ Delivery Destination");

        View dialogView = LayoutInflater.from(this).inflate(android.R.layout.select_dialog_item, null);

        // Buat layout manual (tidak butuh layout baru)
        android.widget.LinearLayout container = new android.widget.LinearLayout(this);
        container.setOrientation(android.widget.LinearLayout.VERTICAL);
        container.setPadding(dpToPx(16), dpToPx(8), dpToPx(16), dpToPx(8));

        EditText etSearch = new EditText(this);
        etSearch.setHint("Type a place name");
        etSearch.setSingleLine(true);
        container.addView(etSearch, new android.widget.LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        TextView tvLoading = new TextView(this);
        tvLoading.setTextSize(11);
        tvLoading.setPadding(0, dpToPx(4), 0, dpToPx(4));
        tvLoading.setVisibility(View.GONE);
        container.addView(tvLoading);

        ListView listView = new ListView(this);
        listView.setDividerHeight(1);
        int listH = dpToPx(200);
        container.addView(listView, new android.widget.LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, listH));

        // Tombol "Klik di Peta"
        Button btnPick = new Button(this);
        btnPick.setText("📍 Tap directly on map");
        btnPick.setTextSize(12);
        container.addView(btnPick, new android.widget.LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        builder.setView(container);
        builder.setNegativeButton("Cancel", null);
        AlertDialog dialog = builder.create();

        final List<NominatimResult> results = new ArrayList<>();
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this,
                android.R.layout.simple_list_item_1, new ArrayList<>());
        listView.setAdapter(adapter);

        Handler uiHandler = new Handler(Looper.getMainLooper());

        // Search saat user ketik (debounce 600ms)
        final Handler[] debounce = {new Handler(Looper.getMainLooper())};
        etSearch.addTextChangedListener(new TextWatcher() {
            @Override public void beforeTextChanged(CharSequence s, int st, int c, int a) {}
            @Override public void onTextChanged(CharSequence s, int st, int b, int c) {
                debounce[0].removeCallbacksAndMessages(null);
                debounce[0].postDelayed(() -> {
                    String q = s.toString().trim();
                    if (q.length() < 3) return;
                    tvLoading.setText("🔍 Searching");
                    tvLoading.setVisibility(View.VISIBLE);
                    searchNominatim(q, found -> uiHandler.post(() -> {
                        results.clear();
                        results.addAll(found);
                        adapter.clear();
                        for (NominatimResult r : found) adapter.add(r.displayName);
                        adapter.notifyDataSetChanged();
                        tvLoading.setVisibility(View.GONE);
                    }));
                }, 600);
            }
            @Override public void afterTextChanged(Editable s) {}
        });

        // Pilih dari hasil list
        listView.setOnItemClickListener((parent, view, pos, id) -> {
            if (pos < results.size()) {
                NominatimResult r = results.get(pos);
                GeoPoint pt = new GeoPoint(r.lat, r.lon);
                if (isPickup) setPickupPoint(pt, r.displayName);
                else          setDestPoint(pt, r.displayName);
                dialog.dismiss();
            }
        });

        // Klik di peta
        btnPick.setOnClickListener(v -> {
            dialog.dismiss();
            if (isPickup) {
                pickingPickup = true;
                btnSetPickup.setText("🟢 Tap on map");
                Toast.makeText(this, "Tap PICKUP point on map", Toast.LENGTH_SHORT).show();
            } else {
                pickingDest = true;
                btnSetDestination.setText("🔵 Tap on map");
                Toast.makeText(this, "Tap DESTINATION point on map", Toast.LENGTH_SHORT).show();
            }
        });

        dialog.show();
    }

    // ── BARU: Nominatim search model ────────────────────────────────────────
    static class NominatimResult {
        String displayName;
        double lat, lon;
        NominatimResult(String name, double lat, double lon) {
            this.displayName = name; this.lat = lat; this.lon = lon;
        }
    }

    interface NominatimCallback {
        void onResult(List<NominatimResult> results);
    }

    // ── BARU: Hit Nominatim API di background thread ─────────────────────
    private void searchNominatim(String query, NominatimCallback callback) {
        executor.execute(() -> {
            List<NominatimResult> out = new ArrayList<>();
            try {
                String encoded = URLEncoder.encode(query, "UTF-8");
                // Nominatim gratis — gunakan User-Agent sesuai app
                URL url = new URL("https://nominatim.openstreetmap.org/search?q="
                        + encoded + "&format=json&limit=8&addressdetails=0");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestProperty("User-Agent", getPackageName());
                conn.setConnectTimeout(8000);
                conn.setReadTimeout(8000);

                BufferedReader br = new BufferedReader(
                        new InputStreamReader(conn.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) sb.append(line);
                br.close();

                JSONArray arr = new JSONArray(sb.toString());
                for (int i = 0; i < arr.length(); i++) {
                    JSONObject obj = arr.getJSONObject(i);
                    String name = obj.getString("display_name");
                    double lat  = obj.getDouble("lat");
                    double lon  = obj.getDouble("lon");
                    out.add(new NominatimResult(name, lat, lon));
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
            callback.onResult(out);
        });
    }

    // ── BARU: Terapkan titik Pickup ─────────────────────────────────────────
    private void setPickupPoint(GeoPoint pt, String name) {
        pickupPoint = pt;
        pickupName  = name.length() > 50 ? name.substring(0, 50) + "…" : name;

        // Hapus marker lama
        if (pickupMarker != null) map.getOverlays().remove(pickupMarker);
        pickupMarker = new Marker(map);
        pickupMarker.setPosition(pt);
        pickupMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
        pickupMarker.setTitle("📦 PICKUP: " + pickupName);
        pickupMarker.setIcon(new BitmapDrawable(getResources(), makePickupBitmap()));
        map.getOverlays().add(pickupMarker);

        // Update info label
        if (tvPickupInfo != null)
            tvPickupInfo.setText("📦 " + pickupName
                    + "\n" + String.format("%.5f, %.5f", pt.getLatitude(), pt.getLongitude()));

        map.getController().animateTo(pt);
        map.invalidate();

        Toast.makeText(this, "Pickup set: " + pickupName, Toast.LENGTH_SHORT).show();

        updateDestinationLabel(); // ← cukup panggil, definisinya sudah ada terpisah di bawah

        if (destPoint != null) autoGenerateWaypoints();
    }

    private void updateDestinationLabel() {
        if (tvDestination == null || isNavigating) return; // saat navigasi, biarkan handleTelemetry() yang update

        StringBuilder sb = new StringBuilder();
        if (pickupPoint != null) {
            sb.append(String.format("WP1 (Pickup): %.6f, %.6f",
                    pickupPoint.getLatitude(), pickupPoint.getLongitude()));
        }
        if (destPoint != null) {
            if (sb.length() > 0) sb.append("\n");
            sb.append(String.format("WP2 (Tujuan): %.6f, %.6f",
                    destPoint.getLatitude(), destPoint.getLongitude()));
        }
        if (sb.length() == 0) {
            sb.append("Move map → ADD POINT");
        }
        tvDestination.setText(sb.toString());
    }

    // ──  Terapkan titik Tujuan ─────────────────────────────────────────
    private void setDestPoint(GeoPoint pt, String name) {
        destPoint = pt;
        destName  = name.length() > 50 ? name.substring(0, 50) + "…" : name;

        // Hapus marker lama
        if (destMarker != null) map.getOverlays().remove(destMarker);
        destMarker = new Marker(map);
        destMarker.setPosition(pt);
        destMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
        destMarker.setTitle("⭐ DESTINATION: " + destName);
        destMarker.setIcon(new BitmapDrawable(getResources(), makeDestBitmap()));
        map.getOverlays().add(destMarker);

        if (tvDestInfo != null)
            tvDestInfo.setText("⭐ " + destName
                    + "\n" + String.format("%.5f, %.5f", pt.getLatitude(), pt.getLongitude()));

        map.getController().animateTo(pt);
        map.invalidate();

        Toast.makeText(this, "Destination set: " + destName, Toast.LENGTH_SHORT).show();

        updateDestinationLabel();
        if (pickupPoint != null) autoGenerateWaypoints();
    }

    // ======================================================================
    //  AUTO GENERATE WAYPOINTS  (robot → pickup → tujuan)
    // ======================================================================
    private void autoGenerateWaypoints() {
        clearWaypointsOnly();

        List<GeoPoint> stops = new ArrayList<>();
        if (robotLat != 0 && robotLon != 0)
            stops.add(new GeoPoint(robotLat, robotLon));
        else
            stops.add(pickupPoint);

        stops.add(pickupPoint);
        stops.add(destPoint);

        fetchOsrmRoute(stops);

        if (tvDestination != null)
            tvDestination.setText("🔄 Calculating route");
    }

    // ── BARU helper: tambah waypoint langsung dari GeoPoint ─────────────────
    private void addWaypointDirect(GeoPoint pos, String label) {
        waypoints.add(pos);

        Marker m = new Marker(map);
        m.setPosition(pos);
        m.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
        m.setTitle("WP " + waypoints.size() + " – " + label);
        m.setIcon(new BitmapDrawable(getResources(), makeWaypointBitmap(waypoints.size())));
        map.getOverlays().add(m);
        waypointMarkers.add(m);
//        drawPlannedRoute();
        map.invalidate();
    }

    private void clearWaypointsOnly() {
        clearWaypointsOnly(true);
    }

    private void clearWaypointsOnly(boolean resetPolyline) {
        for (Marker mk : waypointMarkers) map.getOverlays().remove(mk);
        waypointMarkers.clear();
        waypoints.clear();
        if (resetPolyline && plannedRoute != null)
            plannedRoute.setPoints(new ArrayList<>());
        if (tvWpType != null) tvWpType.setText("WP: -");
        map.invalidate();
    }
    // ======================================================================
    //  TAMBAH WAYPOINT DI TENGAH LAYAR (tombol manual tetap ada)
    // ======================================================================
    private void addWaypointAtCenter() {
        GeoPoint center = (GeoPoint) map.getMapCenter();
        GeoPoint pos    = new GeoPoint(center.getLatitude(), center.getLongitude());
        waypoints.add(pos);

        Marker m = new Marker(map);
        m.setPosition(pos);
        m.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
        m.setTitle("WP " + waypoints.size());
        m.setIcon(new BitmapDrawable(getResources(), makeWaypointBitmap(waypoints.size())));
        map.getOverlays().add(m);
        waypointMarkers.add(m);

        if (tvWpType != null)
            tvWpType.setText("WP: " + waypoints.size());
        if (tvDestination != null && !isNavigating)
            tvDestination.setText(String.format("WP %d  %.6f, %.6f",
                    waypoints.size(), pos.getLatitude(), pos.getLongitude()));

        drawPlannedRoute();
        map.invalidate();
        Toast.makeText(this, "Waypoint " + waypoints.size() + " added", Toast.LENGTH_SHORT).show();
    }

    private void drawPlannedRoute() {
        List<GeoPoint> stops = new ArrayList<>();
        if (robotLat != 0 && robotLon != 0)
            stops.add(new GeoPoint(robotLat, robotLon));
        stops.addAll(waypoints);

        if (stops.size() < 2) {
            if (!stops.isEmpty()) plannedRoute.setPoints(stops);
            map.invalidate();
            return;
        }

        executor.execute(() -> {
            try {
                StringBuilder coords = new StringBuilder();
                for (int i = 0; i < stops.size(); i++) {
                    if (i > 0) coords.append(";");
                    coords.append(stops.get(i).getLongitude())
                            .append(",")
                            .append(stops.get(i).getLatitude());
                }

                String urlStr = "https://router.project-osrm.org/route/v1/driving/"
                        + coords
                        + "?overview=full&geometries=geojson&steps=false";

                URL url = new URL(urlStr);
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestProperty("User-Agent", getPackageName());
                conn.setConnectTimeout(10000);
                conn.setReadTimeout(10000);

                BufferedReader br = new BufferedReader(
                        new InputStreamReader(conn.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) sb.append(line);
                br.close();

                JSONObject root = new JSONObject(sb.toString());
                if (!"Ok".equals(root.optString("code", ""))) return;

                JSONObject route = root.getJSONArray("routes").getJSONObject(0);

                JSONArray coordinates = route
                        .getJSONObject("geometry").getJSONArray("coordinates");

                List<GeoPoint> routePts = new ArrayList<>();
                for (int i = 0; i < coordinates.length(); i++) {
                    JSONArray pt = coordinates.getJSONArray(i);
                    routePts.add(new GeoPoint(pt.getDouble(1), pt.getDouble(0)));
                }

                // ← BARU: ambil distance & duration juga, jangan cuma polyline
                final double finalDist = route.optDouble("distance", 0);
                final double finalDur  = route.optDouble("duration", 0);

                runOnUiThread(() -> {
                    osrmDistanceM   = finalDist;
                    osrmDurationSec = finalDur;
                    plannedRoute.setPoints(routePts);
                    updateEtaDisplay();   // ← langsung tampil tanpa nunggu telemetry
                    map.invalidate();
                });

            } catch (Exception e) {
                runOnUiThread(() -> { plannedRoute.setPoints(stops); map.invalidate(); });
            }
        });
    }
    // ======================================================================
//  OSRM ROUTING — gambar jalur mengikuti jalan, bukan garis lurus
// ======================================================================
    private void fetchOsrmRoute(List<GeoPoint> stops) {
        if (stops.size() < 2) return;

        executor.execute(() -> {
            try {
                StringBuilder coords = new StringBuilder();
                for (int i = 0; i < stops.size(); i++) {
                    if (i > 0) coords.append(";");
                    coords.append(stops.get(i).getLongitude())
                            .append(",")
                            .append(stops.get(i).getLatitude());
                }

                String urlStr = "https://router.project-osrm.org/route/v1/driving/"
                        + coords
                        + "?overview=full&geometries=geojson&steps=true";

                URL url = new URL(urlStr);
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestProperty("User-Agent", getPackageName());
                conn.setConnectTimeout(10000);
                conn.setReadTimeout(10000);

                BufferedReader br = new BufferedReader(
                        new InputStreamReader(conn.getInputStream()));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) sb.append(line);
                br.close();

                JSONObject root = new JSONObject(sb.toString());
                String code = root.optString("code", "");
                if (!"Ok".equals(code)) return;

                JSONObject route = root.getJSONArray("routes").getJSONObject(0);

                JSONArray coordinates = route
                        .getJSONObject("geometry")
                        .getJSONArray("coordinates");

                List<GeoPoint> routePts = new ArrayList<>();
                for (int i = 0; i < coordinates.length(); i++) {
                    JSONArray pt = coordinates.getJSONArray(i);
                    routePts.add(new GeoPoint(pt.getDouble(1), pt.getDouble(0)));
                }

                List<GeoPoint> turningWaypoints = new ArrayList<>();
                JSONArray legs = route.getJSONArray("legs");

                for (int l = 0; l < legs.length(); l++) {
                    JSONArray steps = legs.getJSONObject(l).getJSONArray("steps");

                    for (int s = 0; s < steps.length(); s++) {
                        JSONObject step = steps.getJSONObject(s);
                        JSONObject maneuver = step.getJSONObject("maneuver");
                        String maneuverType = maneuver.optString("type", "");

                        boolean isTurn = maneuverType.equals("turn")
                                || maneuverType.equals("roundabout")
                                || maneuverType.equals("rotary")
                                || maneuverType.equals("fork")
                                || maneuverType.equals("end of road");

                        if (isTurn) {
                            JSONArray loc = maneuver.getJSONArray("location");
                            turningWaypoints.add(new GeoPoint(loc.getDouble(1), loc.getDouble(0)));
                        }
                    }

                    JSONObject lastStep = steps.getJSONObject(steps.length() - 1);
                    JSONArray lastLoc = lastStep.getJSONObject("maneuver").getJSONArray("location");
                    turningWaypoints.add(new GeoPoint(lastLoc.getDouble(1), lastLoc.getDouble(0)));
                }

                double totalDistance = route.optDouble("distance", 0);
                double totalDuration = route.optDouble("duration", 0);

                final double finalDist = totalDistance;
                final double finalDur  = totalDuration;
                final List<GeoPoint> finalRoutePts = routePts;
                final List<GeoPoint> finalTurning  = turningWaypoints;

                runOnUiThread(() -> {
                    osrmDistanceM   = finalDist;
                    osrmDurationSec = finalDur;
                    plannedRoute.setPoints(finalRoutePts);
                    applyOsrmWaypoints(finalTurning);
                    updateEtaDisplay();
                    map.invalidate();
                });

            } catch (Exception e) {
                e.printStackTrace();
                runOnUiThread(() -> {
                    plannedRoute.setPoints(stops);
                    map.invalidate();
                });
            }
        });
    }

    private void applyOsrmWaypoints(List<GeoPoint> turningPoints) {
        clearWaypointsOnly(false);

        for (GeoPoint pt : turningPoints) {
            // tambah marker + data saja, tanpa drawPlannedRoute di dalamnya
            waypoints.add(pt);
            Marker m = new Marker(map);
            m.setPosition(pt);
            m.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
            m.setTitle("WP " + waypoints.size() + " – Turn WP");
            m.setIcon(new BitmapDrawable(getResources(), makeWaypointBitmap(waypoints.size())));
            map.getOverlays().add(m);
            waypointMarkers.add(m);
        }

        updateDestinationLabel();

        if (tvWpType != null)
            tvWpType.setText("WP: " + turningPoints.size());
        map.invalidate();

        Toast.makeText(this,
                "🗺️ " + turningPoints.size() + " turn waypoints found",
                Toast.LENGTH_SHORT).show();
    }
    // ======================================================================
    //  KIRIM WAYPOINT KE ROBOT
    // ======================================================================
    private void sendWaypointToRobot() {
        try {
            pointsKalman.clear();
            pathKalman.setPoints(new ArrayList<>());

            JSONObject payload = new JSONObject();
            JSONArray wpArray  = new JSONArray();
            for (GeoPoint wp : waypoints) {
                JSONObject obj = new JSONObject();
                obj.put("lat", wp.getLatitude());
                obj.put("lon", wp.getLongitude());
                wpArray.put(obj);
            }
            payload.put("waypoints", wpArray);

            // ── BARU: sertakan info pickup & tujuan sebagai metadata ──
            if (pickupPoint != null) {
                JSONObject pkInfo = new JSONObject();
                pkInfo.put("lat",  pickupPoint.getLatitude());
                pkInfo.put("lon",  pickupPoint.getLongitude());
                pkInfo.put("name", pickupName);
                payload.put("pickup", pkInfo);
            }
            if (destPoint != null) {
                JSONObject dstInfo = new JSONObject();
                dstInfo.put("lat",  destPoint.getLatitude());
                dstInfo.put("lon",  destPoint.getLongitude());
                dstInfo.put("name", destName);
                payload.put("destination", dstInfo);
            }

            MqttMessage wpMsg = new MqttMessage(payload.toString().getBytes());
            wpMsg.setQos(1);
            mqttClient.publish(T_WAYPOINT_SET, wpMsg);

            JSONObject cmdPayload = new JSONObject();
            // ── BARU: jika ada pickup → mulai dengan mode PICKUP dulu ──
            cmdPayload.put("mode", pickupPoint != null ? "PICKUP" : "DELIVERY");
            MqttMessage cmdMsg = new MqttMessage(cmdPayload.toString().getBytes());
            cmdMsg.setQos(1);
            mqttClient.publish(T_CMD, cmdMsg);

            isNavigating = true;
            autoFollow   = true;
            String modeStr = pickupPoint != null
                    ? "Pickup → Delivery (2 stages)"
                    : "Direct Delivery";
            if (tvDestination != null)
                tvDestination.setText("Destination : WP 0");
            if (tvWpType != null)
                tvWpType.setText("WP: " + waypoints.size());
            Toast.makeText(this, modeStr + " — " + waypoints.size() + " WP Sent!",
                    Toast.LENGTH_LONG).show();
        } catch (Exception e) { e.printStackTrace(); }
    }

    private void sendSpeed(float spd) {
        if (!mqttReady) return;
        try {
            int pwm = Math.round((spd / 1.6f) * 255);
            JSONObject cmd = new JSONObject();
            cmd.put("cmd",   "set_speed");
            cmd.put("speed", spd);
            cmd.put("value", spd);   // ← TAMBAHAN INI
            cmd.put("pwm",   pwm);
            MqttMessage msg = new MqttMessage(cmd.toString().getBytes());
            msg.setQos(1);
            mqttClient.publish(T_CMD, msg);
            Toast.makeText(this,
                    String.format("Speed %.2f m/s sent", spd),
                    Toast.LENGTH_SHORT).show();
        } catch (Exception e) { e.printStackTrace(); }
    }
    // ======================================================================
    //  UPDATE POSISI ROBOT (EKF)
    // ======================================================================
    private void updateRobotKalman(double lat, double lon, boolean centerMap) {
        robotLat = lat;
        robotLon = lon;
        GeoPoint pos = new GeoPoint(lat, lon);
        robotMarker.setPosition(pos);
        if (isNavigating) {
            pointsKalman.add(pos);
            pathKalman.setPoints(new ArrayList<>(pointsKalman));
        }
        drawPlannedRoute();  // ← OSRM request tiap 1 detik, hapus baris ini
        if (centerMap || autoFollow) map.getController().animateTo(pos);
        map.invalidate();
    }

    // ======================================================================
    //  RESET
    // ======================================================================
    private void resetAll() {
        clearWaypointsOnly();

        // ── BARU: hapus juga pickup & dest ──
        if (pickupMarker != null) { map.getOverlays().remove(pickupMarker); pickupMarker = null; }
        if (destMarker   != null) { map.getOverlays().remove(destMarker);   destMarker   = null; }
        pickupPoint = null; pickupName = "";
        destPoint   = null; destName   = "";
        if (tvPickupInfo != null) tvPickupInfo.setText("📦 Pickup : -");
        if (tvDestInfo   != null) tvDestInfo.setText("⭐ Destination: -");
        btnSetPickup.setText("📦 Set Pickup");
        btnSetDestination.setText("⭐ Set Destination");

        pointsKalman.clear();
        if (pathKalman != null) pathKalman.setPoints(new ArrayList<>());
        isNavigating = false;
        autoFollow   = false;

        if (mqttReady) {
            try {
                JSONObject cmd = new JSONObject();
                cmd.put("cmd", "clear_wp");
                MqttMessage msg = new MqttMessage(cmd.toString().getBytes());
                msg.setQos(1);
                mqttClient.publish(T_CMD, msg);
            } catch (Exception e) { e.printStackTrace(); }
        }

        map.invalidate();
        setStatus("Mode  : Standby", Color.WHITE);
        if (tvDestination != null) tvDestination.setText("Move map → ADD POINT");
        if (tvWpType      != null) tvWpType.setText("WP: -");
    }

    private void updateEtaDisplay() {
        if (tvEtaKalman == null) return;
        if (isNavigating) return; // sudah start → biarkan ETA live dari telemetry yang urus

        if (osrmDistanceM > 0 && currentSpeed > 0.01f) {
            double etaSec = osrmDistanceM / currentSpeed; // ← pakai kecepatan RATA-RATA ROBOT, bukan OSRM
            tvEtaKalman.setText("⏱ " + formatEta(etaSec));
            tvEtaKalman.setTextColor(Color.GRAY);
        } else {
            tvEtaKalman.setText("⏱ - ");
            tvEtaKalman.setTextColor(Color.GRAY);
        }
    }

    // ── BARU: format detik jadi "mm:ss" biar enak dibaca dosen ──
    private String formatEta(double totalSeconds) {
        if (totalSeconds < 60) {
            return String.format("%.0f s", totalSeconds);
        }
        int mins = (int) (totalSeconds / 60);
        int secs = (int) (totalSeconds % 60);
        return String.format("%d menit %02d detik", mins, secs);
    }
    // ======================================================================
    //  MQTT
    // ======================================================================
    private void connectMQTT() {
        String clientId = "AndroidMap_" + UUID.randomUUID().toString().substring(0, 8);
        mqttClient = new MqttAndroidClient(getApplicationContext(), BROKER_URI, clientId);

        mqttClient.setCallback(new MqttCallbackExtended() {
            @Override public void connectComplete(boolean reconnect, String serverURI) {
                mqttReady  = true;
                retryDelay = 3000;
                runOnUiThread(() -> setStatus("Mode  : Connected", Color.GREEN));
                try {
                    mqttClient.subscribe(T_TELEMETRY,    1);
                    mqttClient.subscribe(T_GPS_FILTERED, 1);
                } catch (Exception e) { e.printStackTrace(); }
                resetWatchdog();
            }
            @Override public void connectionLost(Throwable cause) {
                mqttReady = false;
                runOnUiThread(() -> setStatus("Mode : Disconnected", Color.RED));
                stopWatchdog();
                scheduleRetry();
            }
            @Override public void messageArrived(String topic, MqttMessage message) {
                resetWatchdog();
                String payload = new String(message.getPayload());
                try {
                    if (T_TELEMETRY.equals(topic))         handleTelemetry(payload);
                    else if (T_GPS_FILTERED.equals(topic)) handleGpsFiltered(payload);
                } catch (Exception e) { e.printStackTrace(); }
            }
            @Override public void deliveryComplete(IMqttDeliveryToken token) {}
        });

        doConnect();
    }

    private void doConnect() {
        MqttConnectOptions opts = new MqttConnectOptions();
        opts.setAutomaticReconnect(false);
        opts.setCleanSession(true);
        opts.setConnectionTimeout(10);
        opts.setKeepAliveInterval(20);
        runOnUiThread(() -> setStatus("Mode : Connecting...", Color.YELLOW));
        try {
            mqttClient.connect(opts, null, new IMqttActionListener() {
                @Override public void onSuccess(IMqttToken t) {}
                @Override public void onFailure(IMqttToken t, Throwable e) {
                    mqttReady = false;
                    runOnUiThread(() -> setStatus("Mode : Connection Failed", Color.RED));
                    scheduleRetry();
                }
            });
        } catch (Exception e) { e.printStackTrace(); scheduleRetry(); }
    }

    private void scheduleRetry() {
        if (retryRunnable != null) retryHandler.removeCallbacks(retryRunnable);
        retryRunnable = () -> { if (!mqttReady) doConnect(); };
        retryHandler.postDelayed(retryRunnable, retryDelay);
        retryDelay = Math.min(retryDelay * 2, 30_000);
    }

    private void resetWatchdog() {
        stopWatchdog();
        watchdogRunnable = () -> {
            mqttReady = false;
            runOnUiThread(() -> setStatus("Mode : Timeout", Color.RED));
            scheduleRetry();
        };
        watchdogHandler.postDelayed(watchdogRunnable, WATCHDOG_TIMEOUT);
    }

    private void stopWatchdog() {
        if (watchdogRunnable != null) watchdogHandler.removeCallbacks(watchdogRunnable);
    }

    // ======================================================================
    //  HANDLE TELEMETRY
    // ======================================================================
    private void handleTelemetry(String payload) throws Exception {
        JSONObject o = new JSONObject(payload);

        // ── ACK dikirim di sini (thread MQTT, sebelum runOnUiThread) ──
        int pktId = o.optInt("pkt_id", 0);
        if (pktId > 0 && mqttReady) {
            try {
                JSONObject ack = new JSONObject();
                ack.put("id", pktId);
                ack.put("ts", System.currentTimeMillis());
                MqttMessage ackMsg = new MqttMessage(ack.toString().getBytes());
                ackMsg.setQos(1);
                mqttClient.publish("robot/ack", ackMsg);
            } catch (Exception ignored) {}
        }
        // ──────────────────────────────────────────────────────────────

        final double latK    = o.optDouble("lat_kf",    0);
        final double lonK    = o.optDouble("lon_kf",    0);
        final double rem     = o.optDouble("distance",  0);
        final double spd     = o.optDouble("speed",     0);
        final double trv     = o.optDouble("travelled", 0);
        final double wt      = o.optDouble("load",      0);
        final double etaK    = o.optDouble("eta_kf",    0);
        final double tLat    = o.optDouble("tlat",      0);
        final double tLon    = o.optDouble("tlon",      0);
        final int    wpIdx   = o.optInt("wp",           0);
        final int    modeInt = o.optInt("mode",         0);
        final String deliveryStatus = o.optString("delivery_status", "STANDBY");

        runOnUiThread(() -> {
            if (latK != 0.0 && lonK > 100.0) {
                if (!firstFixReceived) {
                    firstFixReceived = true;
                    lastLat = latK; lastLon = lonK;
                    updateRobotKalman(latK, lonK, true);
                } else {
                    double  maxJump = 0.0005;
                    boolean jump    = Math.abs(latK-lastLat) > maxJump
                            || Math.abs(lonK-lastLon) > maxJump;
                    if (!jump) { lastLat = latK; lastLon = lonK; updateRobotKalman(latK, lonK, false); }
                }
            }
            if (tvSpeed != null) {
                tvSpeed.setText(String.format("⚡ %.2f m/s", spd));
                tvSpeed.setTextColor(spd > 0.1 ? Color.parseColor("#00E676") : Color.parseColor("#FF5252"));
            }
            if (tvRemaining != null) tvRemaining.setText(String.format("📍 %.1f m tersisa", rem));
            if (tvTravelled != null) tvTravelled.setText(String.format("🧭 %.1f m ditempuh", trv));
            if (tvWeight    != null) tvWeight.setText(String.format("⚖ %.2f kg", wt));
// ETA 2 mode: live kalau robot jalan, OSRM kalau belum
            final double etaRawVal = o.optDouble("eta_raw", 0);   // ← BARU

            if (tvEtaGps != null && isNavigating) {
                tvEtaGps.setText("⏱ GPS: " + formatEta(etaRawVal));
                tvEtaGps.setTextColor(Color.parseColor("#FFA726"));
            }
            if (tvEtaKalman != null && isNavigating) {
                // [FIX] 0 itu valid (artinya SUDAH SAMPAI), bukan berarti "kosong"
                tvEtaKalman.setText("⏱ EKF: " + formatEta(etaK));
                tvEtaKalman.setTextColor(etaK <= 0.5 ? Color.parseColor("#00E676") : Color.parseColor("#00E5FF"));
            }

            int totalWP = waypoints.size();
            int sisaWP  = (totalWP > 0) ? Math.max(0, totalWP - wpIdx) : 0;
            if (tvWpType != null)
                tvWpType.setText(sisaWP > 0 ? "WP: " + sisaWP : "WP: -");

            if (tvDestination != null && isNavigating) {
                if (tLat != 0 && tLon != 0)
                    tvDestination.setText("Destination : WP " + wpIdx);
            }

            switch (deliveryStatus) {
                case "PICKUP":
                    setStatus("Mode : Pickup 🚚", Color.parseColor("#00E676"));
                    isNavigating = true;
                    break;
                case "DELIVERY":
                    setStatus("Mode : Delivery 🚗", Color.GREEN);
                    isNavigating = true;
                    break;
                default:
                    if (isNavigating) {
                        setStatus("Mode : Done ✓", Color.CYAN);
                        isNavigating = false;
                        if (tvWpType != null) tvWpType.setText("WP: -");
                    } else {
                        setStatus("Mode : Standby", Color.YELLOW);
                    }
                    break;
            }
        });
    }

    // ======================================================================
    //  HANDLE GPS FILTERED
    // ======================================================================
    private void handleGpsFiltered(String payload) throws Exception {
        JSONObject o   = new JSONObject(payload);
        double     lat = o.optDouble("lat", 0);
        double     lon = o.optDouble("lon", 0);
        if (lat == 0 || lon == 0) return;
        runOnUiThread(() -> {
            if (!firstFixReceived) {
                firstFixReceived = true;
                lastLat = lat; lastLon = lon;
                updateRobotKalman(lat, lon, true);
            } else {
                double  maxJump = 0.0005;
                boolean jump    = Math.abs(lat-lastLat) > maxJump || Math.abs(lon-lastLon) > maxJump;
                if (!jump) { lastLat = lat; lastLon = lon; updateRobotKalman(lat, lon, false); }
            }
        });
    }
    // ======================================================================
    //  HELPER
    // ======================================================================
    private void setStatus(String text, int color) {
        if (tvStatus == null) return;
        tvStatus.setText(text); tvStatus.setTextColor(color);
    }

    @Override protected void onResume() {
        super.onResume(); map.onResume();
        firstFixReceived = false;
        if (!mqttReady) doConnect();
    }

    @Override protected void onPause() {
        super.onPause(); map.onPause();
        mqttReady = false;
        try { if (mqttClient != null && mqttClient.isConnected()) mqttClient.disconnect(); }
        catch (Exception ignored) {}
    }

    @Override protected void onDestroy() {
        super.onDestroy();
        stopWatchdog();
        executor.shutdown();
        if (retryRunnable != null) retryHandler.removeCallbacks(retryRunnable);
        mqttReady = false;
        try { if (mqttClient != null && mqttClient.isConnected()) mqttClient.disconnect(); }
        catch (Exception ignored) {}
    }
}