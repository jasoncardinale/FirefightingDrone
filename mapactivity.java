package carobotics.infernofirefightinguav;

import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.IntentFilter;
import android.location.Address;
import android.location.Geocoder;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.io.IOException;
import java.util.List;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback {

   private GoogleMap mMap;

   public Button settings_button;

   public void settingsPage() {
       settings_button = (Button)findViewById(R.id.settings_button);
       settings_button.setOnClickListener(new View.OnClickListener() {
           @Override
           public void onClick(View v) {
               Intent settings_intent = new Intent(MapsActivity.this, SettingsActivity.class);
               startActivity(settings_intent);
           }
       });
   }

   public Button sensor_button;

   public void sensorPage() {
       sensor_button = (Button)findViewById(R.id.sensor_button);
       sensor_button.setOnClickListener(new View.OnClickListener() {
           @Override
           public void onClick(View v) {
               Intent sensor_intent = new Intent(MapsActivity.this, SensorActivity.class);
               startActivity(sensor_intent);
           }
       });
   }

   @Override
   protected void onCreate(Bundle savedInstanceState) {
       super.onCreate(savedInstanceState);
       setContentView(R.layout.activity_maps);
       // Obtain the SupportMapFragment and get notified when the map is ready to be used.
       SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
               .findFragmentById(R.id.map);
       mapFragment.getMapAsync(this);
       settingsPage();
       sensorPage();
   }

   public void onLaunch(View view) {
       EditText location_tf = (EditText)findViewById(R.id.TFaddress);
       String location = location_tf.getText().toString();
       List<Address> addressList = null;
       if(location != null || location.equals("")) {
           Geocoder geocoder = new Geocoder(this);
           try {
               addressList = geocoder.getFromLocationName(location, 1);
           } catch (IOException e) {
               e.printStackTrace();
           }

           Address address = addressList.get(0);
           LatLng latlng = new LatLng(address.getLatitude(), address.getLongitude());
           mMap.addMarker(new MarkerOptions().position(latlng).title("FIRE"));
           mMap.animateCamera(CameraUpdateFactory.newLatLng(latlng));

           Intent fire_coord = new Intent(getApplicationContext(), SettingsActivity.class);
           fire_coord.putExtra("fire", latlng);
       }
   }

   public void onZoom(View view) {
       if(view.getId() == R.id.zoomin) {
           mMap.animateCamera(CameraUpdateFactory.zoomIn());
       }
       if(view.getId() == R.id.zoomout) {
           mMap.animateCamera(CameraUpdateFactory.zoomOut());
       }
   }

   public void changeType(View view) {
      if(mMap.getMapType() == GoogleMap.MAP_TYPE_NORMAL)  {
          mMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);
      }
       else {
          mMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);
      }
   }

   @Override
   public void onMapReady(GoogleMap googleMap) {
       mMap = googleMap;

       int infernoLat = 0;
       int infernoLong = 0;
       LatLng inferno = new LatLng(infernoLat, infernoLong);
       mMap.addMarker(new MarkerOptions().position(inferno).title("Inferno"));
       mMap.moveCamera(CameraUpdateFactory.newLatLng(inferno)); }
   }	
