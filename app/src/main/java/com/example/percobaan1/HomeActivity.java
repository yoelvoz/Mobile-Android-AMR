package com.example.percobaan1;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.WindowManager;
import android.widget.ImageView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.cardview.widget.CardView;

public class HomeActivity extends AppCompatActivity {

    CardView cardMonitor, cardMaps;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        if (getSupportActionBar() != null)
            getSupportActionBar().hide();

        getWindow().setFlags(
                WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS,
                WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS
        );

        setContentView(R.layout.activity_home);

        cardMonitor = findViewById(R.id.cardMonitor);
        cardMaps    = findViewById(R.id.cardMaps);

        View logoContainer = findViewById(R.id.logoContainer);

        ImageView imgRobot = findViewById(R.id.imgRobot);

        animateFadeUp(logoContainer, 0);
        animateFadeUp(cardMonitor, 250);
        animateFadeUp(cardMaps, 450);

        animateFloat(imgRobot);

        cardMonitor.setOnClickListener(v ->
                startActivity(new Intent(this, MainActivity.class)));

        cardMaps.setOnClickListener(v ->
                startActivity(new Intent(this, MapActivity.class)));
    }

    private void animateFadeUp(View view, long delay) {

        view.setAlpha(0f);
        view.setTranslationY(60f);

        view.animate()
                .alpha(1f)
                .translationY(0f)
                .setDuration(700)
                .setStartDelay(delay)
                .start();
    }

    private void animateFloat(View view) {

        view.animate()
                .translationY(-16f)
                .setDuration(1800)
                .withEndAction(() ->
                        view.animate()
                                .translationY(0f)
                                .setDuration(1800)
                                .withEndAction(() -> animateFloat(view))
                                .start()
                )
                .start();
    }
}