package carobotics.infernofirefightinguav;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;

public class SensorActivity extends AppCompatActivity {

   public Button sensor_exit;

   public void mapsPageFromSensor() {
       sensor_exit = (Button)findViewById(R.id.settings_exit);
       sensor_exit.setOnClickListener(new View.OnClickListener() {
           @Override
           public void onClick(View v) {
               Intent sensor_exit_intent = new Intent(SensorActivity.this, MapsActivity.class);
               startActivity(sensor_exit_intent);
           }
       });
   }

   @Override
   protected void onCreate(Bundle savedInstanceState) {
       super.onCreate(savedInstanceState);
       setContentView(R.layout.activity_sensor);
       mapsPageFromSensor();
   }
}
