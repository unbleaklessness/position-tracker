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

class MainActivity :
    AppCompatActivity(),
    SensorEventListener
{

    companion object {
        val TAG = "MainActivity"
    }

    private var sensorManager: SensorManager? = null
    private var accelerometerSensor: Sensor? = null

    private var reconnectButton: Button? = null

    private val serverIP = "10.42.0.1"
    private val serverPort = 7247

    private var tcpClient: TCPClient? = null

    private var lastTimeUpdateInterval: Long = 0
    private var lastTimeDeltaTime: Long = 0
    private val updateInterval: Long = 200000000

    private var px: Double = 0.0
    private var py: Double = 0.0
    private var pz: Double = 0.0
    private var vx: Double = 0.0
    private var vy: Double = 0.0
    private var vz: Double = 0.0

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

        vx += 0.5 * sensorEvent.values[8].toDouble() * dt * dt
        vy += 0.5 * sensorEvent.values[7].toDouble() * dt * dt
        vz += 0.5 * sensorEvent.values[6].toDouble() * dt * dt

        px += vx * dt
        py += vy * dt
        pz += vz * dt

        time = System.nanoTime()
        if (time - lastTimeUpdateInterval > updateInterval) {
            lastTimeUpdateInterval = time

            val dataString = "$px $py $pz\n"
            tcpClient?.sendMessage(dataString)
        }
    }

    private fun disconnect() {
        tcpClient?.close()
        tcpClient = null
    }
}
