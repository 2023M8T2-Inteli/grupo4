from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import orders

app = FastAPI()
app.include_router(orders.router, prefix="/orders")

# Enable CORS
origins = ["*"]  # You might want to restrict this in a production environment
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
