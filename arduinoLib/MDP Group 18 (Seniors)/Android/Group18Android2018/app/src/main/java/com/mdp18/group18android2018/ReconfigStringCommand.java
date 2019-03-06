package com.mdp18.group18android2018;


import android.app.AlertDialog;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.support.v4.content.LocalBroadcastManager;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import java.nio.charset.Charset;
import java.util.UUID;

public class ReconfigStringCommand extends AppCompatActivity{

    private static final String TAG = "ReconfigStringCommand";

    // Declarations
    Button btn_save, btn_reset, btn_retrieve, btn_f1, btn_f2;
    EditText et_f1, et_f2;
    SharedPreferences myPrefs;
    public static final String mypreference="mypref";
    public static final String F1="f1";
    public static final String F2="f2";

    // For bluetooth connection status
    private static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    BluetoothDevice myBTConnectionDevice;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setTitle("Reconfigurable String Commands");
        setContentView(R.layout.activity_reconfig);

        //Register Broadcast Receiver for incoming bluetooth message
        LocalBroadcastManager.getInstance(this).registerReceiver(btConnectionReceiver, new IntentFilter("btConnectionStatus"));

        // GUI Buttons
        btn_save = (Button) findViewById(R.id.btn_save);
        btn_reset = (Button) findViewById(R.id.btn_reset);
        btn_retrieve = (Button) findViewById(R.id.btn_retrieve);
        btn_f1 = (Button) findViewById(R.id.btn_f1);
        btn_f2 = (Button) findViewById(R.id.btn_f2);

        // Use Shared Preferences to save string commands
        myPrefs=getSharedPreferences(mypreference, Context.MODE_PRIVATE);

        init();
        onClickF1();
        onClickF2();

    }

    // On initialisation
    private void init() {
        et_f1 = (EditText) findViewById(R.id.et_f1);
        et_f2 = (EditText) findViewById(R.id.et_f2);
    }

    // Save string commands using Shared Preferences Editor
    public void Save(View view) {
        SharedPreferences.Editor editor = myPrefs.edit();
        editor.putString(F1, et_f1.getText().toString());
        editor.putString(F2, et_f2.getText().toString());
        editor.apply();

        Toast.makeText(this, "Commands saved!", Toast.LENGTH_SHORT).show();
    }

    // Reset saved string commands using Shared Preferences Editor
    public void Reset(View view) {
        SharedPreferences.Editor editor = myPrefs.edit();
        et_f1.setText("");
        et_f2.setText("");
        editor.clear();
        editor.apply();

        Toast.makeText(this, "Commands reset!", Toast.LENGTH_SHORT).show();
    }

    // Retrieve and display previously saved string commands using Shared Preferences
    public void Retrieve (View view) {
        String str_f1 = myPrefs.getString(F1, "");
        et_f1.setText(str_f1);
        String str_f2 = myPrefs.getString(F2, "");
        et_f2.setText(str_f2);
        if (myPrefs.contains(F1)){
            et_f1.setText(myPrefs.getString(F1,"String Not Found"));
        }
        if (myPrefs.contains(F2)){
            et_f2.setText(myPrefs.getString(F2,"String Not Found"));
        }
    }

    // Outgoing message for string command F1
    public void onClickF1(){

        btn_f1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                String tempF1 = myPrefs.getString(F1, "");
                byte[] bytes = tempF1.getBytes(Charset.defaultCharset());
                BluetoothChat.writeMsg(bytes);

                Log.d(TAG, "Outgoing F1 string command: " + tempF1);

                Toast.makeText(ReconfigStringCommand.this, "F1 string command sent.", Toast.LENGTH_SHORT).show();
            }
        });

    }

    // Outgoing message for string command F2
    public void onClickF2() {

        btn_f2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                String tempF2 = myPrefs.getString(F2, "");
                byte[] bytes = tempF2.getBytes(Charset.defaultCharset());
                BluetoothChat.writeMsg(bytes);

                Log.d(TAG, "Outgoing F2 string command: " + tempF2);

                Toast.makeText(ReconfigStringCommand.this, "F2 string command sent.", Toast.LENGTH_SHORT).show();
            }
        });

    }


    /* MENU FOR RECONFIGURABLE STRING COMMANDS*/

    @Override
    public boolean onCreateOptionsMenu (Menu menu) {
        // Inflate the menu if present
        getMenuInflater().inflate(R.menu.menu_reconfig, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected (MenuItem item) {
        switch (item.getItemId()){
            case R.id.btn_bluetoothconnect:
                Intent intent = new Intent(ReconfigStringCommand.this, BluetoothConnect.class);
                intent.addFlags(Intent.FLAG_ACTIVITY_REORDER_TO_FRONT);
                startActivity(intent);
                return true;

            case R.id.btn_mainscreen:
                startActivity(new Intent(ReconfigStringCommand.this,MainActivity.class));
                return true;
        }
        return super.onOptionsItemSelected(item);

    }

    // Return to main screen after back button is selected.
    @Override
    public void onBackPressed() {
        Intent intent = new Intent(ReconfigStringCommand.this, MainActivity.class);
        intent.addFlags(Intent.FLAG_ACTIVITY_REORDER_TO_FRONT);
        startActivity(intent);
    }


    //Broadcast Receiver for Bluetooth Connection Status (Robust BT Connection)
    BroadcastReceiver btConnectionReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {

            Log.d(TAG, "Receiving btConnectionStatus Msg!!!");

            String connectionStatus = intent.getStringExtra("ConnectionStatus");
            myBTConnectionDevice = intent.getParcelableExtra("Device");

            // IF disconnected from Bluetooth
            if(connectionStatus.equals("disconnect")){

                Log.d("ConnectAcitvity:","Device Disconnected");

                //Stop Bluetooth Connection Service

                // Reconnect Bluetooth
                AlertDialog alertDialog = new AlertDialog.Builder(ReconfigStringCommand.this).create();
                alertDialog.setTitle("BLUETOOTH DISCONNECTED");
                alertDialog.setMessage("Connection with device: '"+myBTConnectionDevice.getName()+"' has ended. Do you want to reconnect?");
                alertDialog.setButton(AlertDialog.BUTTON_POSITIVE, "Yes",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int which) {

                                // Start Bluetooth Connection Service
                                Intent connectIntent = new Intent(ReconfigStringCommand.this, BluetoothConnectionService.class);
                                connectIntent.putExtra("serviceType", "connect");
                                connectIntent.putExtra("device", myBTConnectionDevice);
                                connectIntent.putExtra("id", myUUID);
                                startService(connectIntent);

                            }
                        });
                alertDialog.setButton(AlertDialog.BUTTON_NEGATIVE, "No",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int which) {
                                dialog.dismiss();
                            }
                        });

                if(!isFinishing()){
                    alertDialog.show();
                }
            }

            // Connected to Bluetooth
            else if(connectionStatus.equals("connect")){


                Log.d("ConnectAcitvity:","Device Connected");
                Toast.makeText(ReconfigStringCommand.this, "Connection Established: "+ myBTConnectionDevice.getName(),
                        Toast.LENGTH_LONG).show();
            }

            // Connection to Bluetooth failed
            else if(connectionStatus.equals("connectionFail")) {
                Toast.makeText(ReconfigStringCommand.this, "Connection Failed: "+ myBTConnectionDevice.getName(),
                        Toast.LENGTH_LONG).show();
            }

        }
    };
}
