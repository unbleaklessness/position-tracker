package com.sorokinoleg.positiontracker

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.AsyncTask
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.widget.Button
import org.apache.commons.math3.filter.*

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

    private var kalman: KalmanFilter? = null

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

        kalman = getKalman()

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

        val data = doubleArrayOf(
            sensorEvent.values[0].toDouble(),
            sensorEvent.values[1].toDouble(),
            sensorEvent.values[2].toDouble()
        )

        kalman?.predict()
        kalman?.correct(data)

        val estimation = kalman?.stateEstimation

        Log.i("Estimation", estimation?.size.toString())

        estimation?.let {
            x = estimation[0]
            y = estimation[1]
            z = estimation[2]
        }

        time = System.nanoTime()
        if (time - lastTimeUpdateInterval > updateInterval) {
            lastTimeUpdateInterval = time

            val dataString = "$x $y $z\n"
            tcpClient?.sendMessage(dataString)
        }
    }

    private fun getKalman(): KalmanFilter {

        val dt = 0.01
        val ele = 0.5 * dt * dt
        val processModel = DefaultProcessModel(
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, dt, 0.0, 0.0, ele, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, ele, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, ele),
                doubleArrayOf(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            ),
            arrayOf(
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0)
            ),
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            )
        )

        val measurementNoise = 1.0
        val measurementModel = DefaultMeasurementModel(
            arrayOf(
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            ),
            arrayOf(
                doubleArrayOf(measurementNoise, 0.0, 0.0),
                doubleArrayOf(0.0, measurementNoise, 0.0),
                doubleArrayOf(0.0, 0.0, measurementNoise)
            )
        )

        return KalmanFilter(processModel, measurementModel)
    }

    private fun disconnect() {
        tcpClient?.close()
        tcpClient = null
    }
}
