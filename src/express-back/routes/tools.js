const express = require('express');
const router = express.Router();
const prisma = require('../config');

// Read - Get all tools
router.get('/', async (req, res) => {
    try {
        const tools = await prisma.tool.findMany();

        res.json(tools);
    } catch (error) {
        console.error('Error fetching tools:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Read - Get a specific tool by ID
router.get('/:id', async (req, res) => {
    const { id } = req.params;

    try {
        const tool = await prisma.tool.findUnique({
            where: {
                id,
            },
        });

        if (!tool) {
            return res.status(404).json({ error: 'Tool not found' });
        }

        res.json(tool);
    } catch (error) {
        console.error('Error fetching tool:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Create - Add a new tool
router.post('/', async (req, res) => {
    const { name, price, tag, minQuantity, maxQuantity } = req.body;

    try {
        const tool = await prisma.tool.create({
            data: {
                name,
                price,
                tag,
                minQuantity,
                maxQuantity,
            },
        });

        res.json(tool);
    } catch (error) {
        console.error('Error creating tool:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Delete all
router.delete('/', async (req, res) => {
    try {
        const tools = await prisma.tool.deleteMany();

        res.json(tools);
    } catch (error) {
        console.error('Error deleting tools:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

router.put('/:id', async (req, res) => {
    const { id } = req.params;
    const { name, price, tag, minQuantity, maxQuantity } = req.body;

    try {
        const existingTool = await prisma.tool.findUnique({
            where: {
                id,
            },
        });

        if (!existingTool) {
            return res.status(404).json({ error: 'Tool not found' });
        }

        const updatedTool = await prisma.tool.update({
            where: {
                id,
            },
            data: {
                name: name || existingTool.name,
                price: parseFloat(price) || existingTool.price,
                tag: tag || existingTool.tag,
                minQuantity: parseFloat(minQuantity) || existingTool.minQuantity,
                maxQuantity: parseFloat(maxQuantity) || existingTool.maxQuantity,
            },
        });

        res.json(updatedTool);
    } catch (error) {
        console.error('Error updating tool:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

module.exports = router;