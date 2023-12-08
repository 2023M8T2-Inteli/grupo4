const express = require("express");
const router = express.Router();
const prisma = require("../config");

// Read - Get all orders in progress
router.get("/queue", async (req, res) => {
  try {
    const orders = await prisma.order.findMany({
      where: {
        type: "In Progress",
      },
      include: {
        tool: {
          select: {
            name: true,
          },
        },
        user: {
          select: {
            name: true,
          },
        },
        point: {
          select: {
            name: true,
          },
        },
      },
      orderBy: {
        createdAt: "asc", // or 'desc' for descending order
      },
    });
    res.json(orders);
  } catch (error) {
    console.error("Error fetching queue data:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

router.get("/history", async (req, res) => {
  console.log("hit");
  try {
    const orders = await prisma.order.findMany({
      where: {
        type: "Completed",
      },
      include: {
        tool: {
          select: {
            name: true,
          },
        },
        user: {
          select: {
            name: true,
          },
        },
        point: {
          select: {
            name: true,
          },
        },
      },
      orderBy: {
        createdAt: "asc", // or 'desc' for descending order
      },
    });
    console.log(orders);
    res.json(orders);
  } catch (error) {
    console.error("Error fetching queue data:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

// Read - Get all orders
router.get("/all", async (req, res) => {
  try {
    const orders = await prisma.order.findMany({
      include: {
        tool: {
          select: {
            name: true,
          },
        },
        user: {
          select: {
            name: true,
          },
        },
        point: {
          select: {
            name: true,
          },
        },
      },
      orderBy: {
        createdAt: "asc", // or 'desc' for descending order
      },
    });
    res.json(orders);
  } catch (error) {
    console.error("Error fetching queue data:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
}
);

// Read - Get a specific order by ID
router.get("/queue/:id", async (req, res) => {
  const { id } = req.params;

  try {
    const order = await prisma.order.findUnique({
      where: {
        id,
      },
    });

    if (!order) {
      return res.status(404).json({ error: "Order not found" });
    }

    res.json(order);
  } catch (error) {
    console.error("Error fetching order:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

// Create - Add a new order
router.post("/queue", async (req, res) => {
  const { toolId, userId, pointId, type } = req.body;

  try {
    const order = await prisma.order.create({
      data: {
        type: type || "In Progress",
        toolId,
        userId,
        pointId,
      },
    });

    res.json(order);
  } catch (error) {
    console.error("Error creating order:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

// Update - Update an existing order
router.put("/queue/:id", async (req, res) => {
  const { id } = req.params;
  const { toolId, userId, pointId } = req.body;

  try {
    const order = await prisma.order.update({
      where: {
        id,
      },
      data: {
        toolId,
        userId,
        pointId,
      },
    });

    res.json(order);
  } catch (error) {
    console.error("Error updating order:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

// Delete - Delete an existing order
router.delete("/queue/:id", async (req, res) => {
  const { id } = req.params;

  try {
    const order = await prisma.order.delete({
      where: {
        id,
      },
    });

    res.json(order);
  } catch (error) {
    console.error("Error deleting order:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

// Delete all
router.delete("/", async (req, res) => {
  try {
    const orders = await prisma.order.deleteMany();

    res.json(orders);
  } catch (error) {
    console.error("Error deleting orders:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

module.exports = router;
