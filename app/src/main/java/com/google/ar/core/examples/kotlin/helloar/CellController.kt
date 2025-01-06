package com.google.ar.core.examples.kotlin.helloar


import com.google.ar.core.Plane
import com.google.ar.core.Pose
import com.google.ar.core.Anchor
import java.util.*

class CellController(val plane: Plane, private val stepSize: Float = 0.1f) {

    val cellMap = mutableMapOf<Pair<Int, Int>, Cell>() // Map
    val edgeQueue: Queue<Cell> = LinkedList()          // Edge Queue for BFS

    fun initialize() {
        // start cell (0, 0)
        val startPose = plane.centerPose
        val startAnchor = plane.createAnchor(startPose)
        val startCell = Cell(0, 0, startAnchor)
        cellMap[startCell.xIndex to startCell.zIndex] = startCell
        edgeQueue.add(startCell)
    }

    fun expandGrid() {
        edgeQueue.removeIf { cell ->
            cell.neighbors.all { cellMap.containsKey(it) }
        }

        var edgeQueueCopy = LinkedList(edgeQueue)

        val size = edgeQueueCopy.size

        repeat(size) {
            val current = edgeQueueCopy.poll() ?: return@repeat

            current.neighbors.forEach { neighbor ->
                addNeighbor(neighbor)
            }
        }
    }

    private fun addNeighbor(key: Pair<Int, Int>) {

        val xIndex = key.first
        val zIndex = key.second

        if (!cellMap.containsKey(key)) {
            val newPose = Pose.makeTranslation(xIndex * stepSize, 0f, zIndex * stepSize)
            if (plane.isPoseInPolygon(newPose)) {
                val newAnchor = plane.createAnchor(newPose)
                val newCell = Cell(xIndex, zIndex, newAnchor)
                cellMap[key] = newCell
                edgeQueue.add(newCell)
            }
        }
    }

    fun drawCells() {
        cellMap.values.forEach { cell ->
            // todo
        }
    }

    fun releaseAllCells() {
        cellMap.values.forEach { it.anchor.detach() }
        cellMap.clear()
        edgeQueue.clear()
    }
}

data class Cell(val xIndex: Int, val zIndex: Int, val anchor: Anchor) {
    var completed: Boolean = false
    val neighbors: List<Pair<Int, Int>>
        get() = listOf(
            Pair(xIndex + 1, zIndex), // right
            Pair(xIndex - 1, zIndex), // left
            Pair(xIndex, zIndex + 1), // front
            Pair(xIndex, zIndex - 1)  // back
        )
}