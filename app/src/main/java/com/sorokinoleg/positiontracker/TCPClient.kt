package com.sorokinoleg.positiontracker

import android.util.Log
import java.io.*
import java.lang.Exception
import java.net.InetAddress
import java.net.Socket

class TCPClient(private val serverIP: String, private val serverPort: Int, private var listener: OnMessageReceived?) {

    companion object {
        val TAG = "TCPClient"
    }

    private var running = false

    private var out: PrintWriter? = null
    private var input: BufferedReader? = null

    private var message: String? = null

    interface OnMessageReceived {
        fun messageReceived(message: String)
    }

    fun run() {
        running = true

        try {
            val serverAddress = InetAddress.getByName(serverIP)
            val socket = Socket(serverAddress, serverPort)

            try {
                out = PrintWriter(BufferedWriter(OutputStreamWriter(socket.getOutputStream())), true)
                input = BufferedReader(InputStreamReader(socket.getInputStream()))

                while (running) {
                    message = input?.readLine()

                    if (message != null && listener != null) {
                        listener!!.messageReceived(message!!)
                    }

                    message = null
                }
            } catch (e: Exception) {
                Log.e("TCP", "Error", e)
            } finally {
                socket.close()
            }
        } catch (e: Exception) {
            Log.e("TCP", "Error", e)
        }
    }

    fun close() {
        running = false

        out?.flush()
        out?.close()

        out = null
        input = null
        listener = null
        message = null
    }

    fun sendMessage(message: String) {
        Thread(Runnable {
            out?.let {
                if (!it.checkError()) {
                    it.print(message)
                    it.flush()
                }
            }
        }).start()
    }
}