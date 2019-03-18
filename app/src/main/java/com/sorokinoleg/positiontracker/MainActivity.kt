package com.sorokinoleg.positiontracker

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.AsyncTask
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.widget.Button
import android.widget.TextView

class MainActivity :
    AppCompatActivity(),
    SensorEventListener
{

    companion object {
        val TAG = "MainActivity"
    }

    private var sensorManager: SensorManager? = null
    private var accelerometerSensor: Sensor? = null

    private var accelerometerTextView: TextView? = null
    private var reconnectButton: Button? = null

    private val serverIP = "10.42.0.1"
    private val serverPort = 7247

    private var tcpClient: TCPClient? = null

    private var lastTimeUpdateInterval: Long = 0
    private var lastTimeDeltaTime: Long = 0
    private val updateInterval: Long = 300000000

    private var x: Double = 0.0
    private var y: Double = 0.0
    private var z: Double = 0.0

    inner class ConnectTask : AsyncTask<Unit, Unit, Unit>() {

        override fun doInBackground(vararg params: Unit?) {

            tcpClient = TCPClient(serverIP, serverPort, null)
            tcpClient?.run()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        ConnectTask().execute()

        lastTimeUpdateInterval = System.nanoTime()
        lastTimeDeltaTime = System.nanoTime()

        accelerometerTextView = findViewById(R.id.accelerometer_text_view)
        reconnectButton = findViewById(R.id.reconnect_button)

        reconnectButton?.setOnClickListener {
            disconnect()
            ConnectTask().execute()
        }

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as? SensorManager
        accelerometerSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)

        sensorManager?.registerListener(this, accelerometerSensor, SensorManager.SENSOR_DELAY_FASTEST)
    }

    override fun onPause() {
        super.onPause()

        disconnect()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Do nothing.
    }

    override fun onSensorChanged(event: SensorEvent?) {
        val sensorEvent = event!!


        var time = System.nanoTime()
        val dt: Double = (time - lastTimeDeltaTime) / 1000000000.0
        lastTimeDeltaTime = time

        x += sensorEvent.values[0] * dt
        y += sensorEvent.values[1] * dt
        z += sensorEvent.values[2] * dt

        time = System.nanoTime()
        if (time - lastTimeUpdateInterval > updateInterval) {
            lastTimeUpdateInterval = time

            val dataString = "$x $y $z\n"
            setAccelerometerText(dataString)
            tcpClient?.sendMessage(dataString)

            x = 0.0
            y = 0.0
            z = 0.0
        }
    }

    private fun disconnect() {
        tcpClient?.close()
        tcpClient = null
    }


    private fun setAccelerometerText(text: String) {
        accelerometerTextView?.text = "Accelerometer: $text"
    }
}
