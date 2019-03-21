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

    private val nanoSeconds: Double = 1000000000.0

    private var lastTimeSendInterval: Long = 0
    private var lastTimeUpdateInterval: Long = 0
    private val sendInterval: Long = 200000000
    private val updateInterval: Long = 1000

    private var kalmanX: KalmanFilter? = null
    private var kalmanY: KalmanFilter? = null
    private var kalmanZ: KalmanFilter? = null

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

        initializeKalmans()

        lastTimeSendInterval = System.nanoTime()
        lastTimeUpdateInterval = System.nanoTime()

        reconnectButton = findViewById(R.id.reconnect_button)
        reconnectButton?.setOnClickListener {
            disconnect()
            ConnectTask().execute()
        }

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as? SensorManager
        accelerometerSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)

        sensorManager?.registerListener(this, accelerometerSensor, SensorManager.SENSOR_DELAY_FASTEST)

        ConnectTask().execute()
    }

    override fun onPause() {
        super.onPause()

        disconnect()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Do nothing.
    }

    override fun onSensorChanged(event: SensorEvent?) {
        event?.let { sensorEvent ->
            var time: Long = System.nanoTime()
            if (time - lastTimeUpdateInterval > updateInterval) {
                lastTimeUpdateInterval = time

                val values = sensorEvent.values.map { it.toDouble() }
                predict(values[0], values[1], values[2])
            }

            time = System.nanoTime()
            if (time - lastTimeSendInterval > sendInterval) {
                lastTimeSendInterval = time

                tcpClient?.sendMessage("$x $y $z\n")
            }
        }
    }

    private fun predict(nx: Double, ny: Double, nz: Double) {

        kalmanX?.let {
            it.predict()
            it.correct(doubleArrayOf(nx))
            x = it.stateEstimation[0]
        }

        kalmanY?.let {
            it.predict()
            it.correct(doubleArrayOf(ny))
            y = it.stateEstimation[0]
        }

        kalmanZ?.let {
            it.predict()
            it.correct(doubleArrayOf(nz))
            z = it.stateEstimation[0]
        }
    }

    private fun createKalman(): KalmanFilter {

        val linearAccelerationError = 0.59820562601
        val s = linearAccelerationError * linearAccelerationError

        val n = 1.2

        val dt: Double = updateInterval / nanoSeconds
        val a = 0.5 * dt * dt

        val processModel = DefaultProcessModel(
            arrayOf(
                doubleArrayOf(1.0, dt, a),
                doubleArrayOf(0.0, 1.0, dt),
                doubleArrayOf(0.0, 0.0, 1.0)
            ),
            arrayOf(
                doubleArrayOf(0.0),
                doubleArrayOf(0.0),
                doubleArrayOf(0.0)
            ),
            arrayOf(
                doubleArrayOf(1.0, 1.0, n),
                doubleArrayOf(1.0, n, n * n),
                doubleArrayOf(n, n * n, n * n * n)
            )
        )

        val measurementModel = DefaultMeasurementModel(
            arrayOf(
                doubleArrayOf(0.0, 0.0, 1.0)
            ),
            arrayOf(
                doubleArrayOf(s)
            )
        )

        return KalmanFilter(processModel, measurementModel)
    }

    private fun initializeKalmans() {
        kalmanX = createKalman()
        kalmanY = createKalman()
        kalmanZ = createKalman()
    }

    private fun disconnect() {
        tcpClient?.close()
        tcpClient = null
    }
}
