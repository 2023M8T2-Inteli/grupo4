const express = require('express');
const router = express.Router();
const prisma = require('../config');

// Read - Get all points
router.get('/', async (req, res) => {
    try {
        const points = await prisma.point.findMany();

        res.json(points);
    } catch (error) {
        console.error('Error fetching points:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Read - Get a specific point by ID
router.get('/:id', async (req, res) => {
    const { id } = req.params;

    try {
        const point = await prisma.point.findUnique({
            where: {
                id,
            },
        });

        if (!point) {
            return res.status(404).json({ error: 'Point not found' });
        }

        res.json(point);
    } catch (error) {
        console.error('Error fetching point:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Create - Add a new point
router.post('/', async (req, res) => {
    let { name, pointX, pointY, pointZ } = req.body;
    pointX = parseFloat(pointX);
    pointY = parseFloat(pointY);
    pointZ = parseFloat(pointZ);

    try {
        const point = await prisma.point.create({
            data: {
                name,
                pointX,
                pointY,
                pointZ,
            },
        });

        res.json(point);
    } catch (error) {
        console.error('Error creating point:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Delete all
router.delete('/', async (req, res) => {
    try {
        const points = await prisma.point.deleteMany();

        res.json(points);
    } catch (error) {
        console.error('Error deleting points:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Delete - Delete a specific point by ID
router.delete('/:id', async (req, res) => {
    const { id } = req.params;

    try {
        const deletedPoint = await prisma.point.delete({
            where: {
                id,
            },
        });

        res.json(deletedPoint);
    } catch (error) {
        console.error('Error deleting point:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Update - Update a specific point by ID
router.put('/:id', async (req, res) => {
    const { id } = req.params;
    const { name, pointX, pointY, pointZ } = req.body;

    try {
        const existingPoint = await prisma.point.findUnique({
            where: {
                id,
            },
        });

        if (!existingPoint) {
            return res.status(404).json({ error: 'Point not found' });
        }

        const updatedPoint = await prisma.point.update({
            where: {
                id,
            },
            data: {
                name: name || existingPoint.name,
                pointX: parseFloat(pointX) || existingPoint.pointX,
                pointY: parseFloat(pointY) || existingPoint.pointY,
                pointZ: parseFloat(pointZ) || existingPoint.pointZ,
            },
        });

        res.json(updatedPoint);
    } catch (error) {
        console.error('Error updating point:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

module.exports = router;