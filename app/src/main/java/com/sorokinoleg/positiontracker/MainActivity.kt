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
import org.apache.commons.math3.filter.*
import kotlin.math.pow

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
    private var resetButton: Button? = null
    private var statusTextView: TextView? = null

    private val serverIP = "10.42.0.1"
    private val serverPort = 7247

    private var tcpClient: TCPClient? = null

    private val nanoSecondsInSecond: Long = 1000000000

    private var lastTimeSendInterval: Long = 0
    private var lastTimeUpdateInterval: Long = 0
    private val sendInterval: Long = 1000000
    private var updateInterval: Long = 5000000

    private var stateSize = 3

    private var kalmanList = Array<KalmanFilter?>(stateSize) { null }
    private var output = DoubleArray(stateSize)

    private var dt: Double = 0.0

    // For average delta time calculation:
    private var averageDeltaTimeCalculated = false
    private var lastTimeDeltaTime: Long = 0
    private var deltaTimeListSize = 1000
    private var deltaTimeList = LongArray(deltaTimeListSize)
    private var deltaTimeListCount = 0

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

        val time = System.nanoTime()
        lastTimeSendInterval = time
        lastTimeUpdateInterval = time
        lastTimeDeltaTime = time

        statusTextView = findViewById(R.id.status_text_view)

        resetButton = findViewById(R.id.reset_button)
        resetButton?.setOnClickListener {
            reset()
        }

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

        // Calculate average delta time between sensor updates and reinitialize everything with this average value:
        if (deltaTimeListCount < deltaTimeListSize) {

            val time = System.nanoTime()
            deltaTimeList[deltaTimeListCount] = time - lastTimeDeltaTime
            lastTimeDeltaTime = time

            if (deltaTimeListCount == deltaTimeListSize - 1) {
                val average = deltaTimeList.average()
                dt = average / nanoSecondsInSecond
                updateInterval = average.toLong()
                averageDeltaTimeCalculated = true
                reset()
            }

            deltaTimeListCount++
        }

        event?.let { sensorEvent ->

            // Update cached data with measurements, filtered with Kalman:
            var time: Long = System.nanoTime()
            if (time - lastTimeUpdateInterval > updateInterval) {
                lastTimeUpdateInterval = time

                predict(sensorEvent.values.map { it.toDouble() }.toDoubleArray())
            }

            // Send cached data:
            time = System.nanoTime()
            if (time - lastTimeSendInterval > sendInterval && averageDeltaTimeCalculated) {
                lastTimeSendInterval = time

                tcpClient?.sendMessage("${output[0]} ${output[1]} ${output[2]}\n")
            }
        }
    }

    private fun reset() {
        initializeKalmans()
        output = output.map { 0.0 }.toDoubleArray()
    }

    private fun predict(data: DoubleArray) {

        for (index in 0 until stateSize) {
            kalmanList[index]?.let {
                it.predict()
                it.correct(doubleArrayOf(data[index]))
                output[index] = it.stateEstimation[0]
            }
        }
    }

    private fun createKalman(): KalmanFilter {

        val linearAccelerationError = 0.59820562601

        val s = linearAccelerationError.pow(2)
        val n = 0.000001
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
            arrayOf(doubleArrayOf(0.0, 0.0, 1.0)),
            arrayOf(doubleArrayOf(s))
        )

        return KalmanFilter(processModel, measurementModel)
    }

    private fun initializeKalmans() {
        for (index in 0 until stateSize) {
            kalmanList[index] = createKalman()
        }
    }

    private fun disconnect() {
        tcpClient?.close()
        tcpClient = null
    }
}
