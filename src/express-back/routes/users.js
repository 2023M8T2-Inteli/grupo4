const express = require('express');
const router = express.Router();
const prisma = require('../config');

// Read - Get all users
router.get('/', async (req, res) => {
    try {
        const users = await prisma.user.findMany();

        res.json(users);
    } catch (error) {
        console.error('Error fetching users:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Read - Get a specific user by ID
router.get('/:id', async (req, res) => {
    const { id } = req.params;

    try {
        const user = await prisma.user.findUnique({
            where: {
                id,
            },
        });

        if (!user) {
            return res.status(404).json({ error: 'User not found' });
        }

        res.json(user);
    } catch (error) {
        console.error('Error fetching user:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Create - Add a new user
router.post('/', async (req, res) => {
    const { name, cellPhone } = req.body;

    try {
        const user = await prisma.user.create({
            data: {
                name,
                cellPhone,
            },
        });

        res.json(user);
    } catch (error) {
        console.error('Error creating user:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// Delete all
router.delete('/', async (req, res) => {
    try {
        const users = await prisma.user.deleteMany();

        res.json(users);
    } catch (error) {
        console.error('Error deleting users:', error);
        res.status(500).json({ error: 'Internal Server Error' });
    }
});

// PUT route to update user data
router.put("/:id",async (req, res) => {
    const { id } = req.params;
    let { name, cellPhone, role } = req.body;
  
    try {
        const user = await prisma.user.findUnique({
          where: { id: id },
        });
    
        if (!user) {
          return res.status(404).json({ message: "User not found" });
        }

        if (role) {
            role = [role]
        }
    
        // Update the user data
        const updatedUser = await prisma.user.update({
          where: { id: id },
          data: {
            name: name || user.name,
            cellPhone: cellPhone || user.cellPhone,
            role: role || user.role,
          },
        });
    
        res.json({ message: "User updated successfully", user: updatedUser });
      } catch (error) {
        console.error("Error updating user:", error);
        res.status(500).json({ message: "Internal server error" });
      }
    });
  

module.exports = router;