from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import Optional
from sqlalchemy.orm import Session
import bcrypt
from ..database import get_db
from ..models.user import User

router = APIRouter()

class UserRegistration(BaseModel):
    email: str
    password: str
    full_name: str
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    robotics_background: Optional[str] = None
    ai_ml_experience: Optional[str] = None

class UserLogin(BaseModel):
    email: str
    password: str

class UserResponse(BaseModel):
    id: int
    email: str
    full_name: Optional[str]
    software_experience: Optional[str]
    hardware_experience: Optional[str]
    robotics_background: Optional[str]
    ai_ml_experience: Optional[str]

    class Config:
        from_attributes = True

@router.post("/register", response_model=UserResponse)
async def register_user(user_data: UserRegistration, db: Session = Depends(get_db)):
    """Register a new user with background information"""
    # Check if user already exists
    existing_user = db.query(User).filter(User.email == user_data.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="User with this email already exists"
        )

    # Hash the password
    hashed_password = bcrypt.hashpw(user_data.password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

    # Create new user
    db_user = User(
        email=user_data.email,
        hashed_password=hashed_password,
        full_name=user_data.full_name,
        software_experience=user_data.software_experience,
        hardware_experience=user_data.hardware_experience,
        robotics_background=user_data.robotics_background,
        ai_ml_experience=user_data.ai_ml_experience
    )

    db.add(db_user)
    db.commit()
    db.refresh(db_user)

    return db_user

@router.post("/login")
async def login_user(credentials: UserLogin, db: Session = Depends(get_db)):
    """Login user and return authentication token"""
    # Find user by email
    user = db.query(User).filter(User.email == credentials.email).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Verify password
    if not bcrypt.checkpw(credentials.password.encode('utf-8'), user.hashed_password.encode('utf-8')):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # In a real implementation, you would generate and return a JWT token
    # For now, we'll just return a success message
    return {
        "message": "Login successful",
        "user_id": user.id,
        "email": user.email
    }

@router.get("/profile/{user_id}", response_model=UserResponse)
async def get_user_profile(user_id: int, db: Session = Depends(get_db)):
    """Get user profile information"""
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    return user