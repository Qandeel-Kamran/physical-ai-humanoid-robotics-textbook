import React, { useState } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

const AuthModal = ({ isOpen, onClose, onLogin, onRegister }) => {
  const [isLogin, setIsLogin] = useState(true);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    fullName: '',
    softwareExperience: '',
    hardwareExperience: '',
    roboticsBackground: '',
    aiMlExperience: ''
  });
  const [isLoading, setIsLoading] = useState(false);
  const { colorMode } = useColorMode();

  if (!isOpen) return null;

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);

    try {
      if (isLogin) {
        // Login logic
        await onLogin(formData.email, formData.password);
      } else {
        // Registration logic with background questions
        await onRegister({
          email: formData.email,
          password: formData.password,
          fullName: formData.fullName,
          softwareExperience: formData.softwareExperience,
          hardwareExperience: formData.hardwareExperience,
          roboticsBackground: formData.roboticsBackground,
          aiMlExperience: formData.aiMlExperience
        });
      }
      onClose();
    } catch (error) {
      console.error('Authentication error:', error);
      alert(`Authentication failed: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`auth-modal-overlay ${colorMode}`}>
      <div className={`auth-modal ${colorMode}`}>
        <div className="auth-modal-header">
          <h2>{isLogin ? 'Login' : 'Register'}</h2>
          <button className="auth-modal-close" onClick={onClose}>
            ×
          </button>
        </div>

        <form onSubmit={handleSubmit} className="auth-form">
          {!isLogin && (
            <div className="form-group">
              <label htmlFor="fullName">Full Name</label>
              <input
                type="text"
                id="fullName"
                name="fullName"
                value={formData.fullName}
                onChange={handleChange}
                required={!isLogin}
                disabled={isLoading}
              />
            </div>
          )}

          <div className="form-group">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
              disabled={isLoading}
            />
          </div>

          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              required
              disabled={isLoading}
            />
          </div>

          {!isLogin && (
            <>
              <div className="form-group">
                <label htmlFor="softwareExperience">Software Experience</label>
                <select
                  id="softwareExperience"
                  name="softwareExperience"
                  value={formData.softwareExperience}
                  onChange={handleChange}
                  disabled={isLoading}
                >
                  <option value="">Select your experience level</option>
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="hardwareExperience">Hardware Experience</label>
                <select
                  id="hardwareExperience"
                  name="hardwareExperience"
                  value={formData.hardwareExperience}
                  onChange={handleChange}
                  disabled={isLoading}
                >
                  <option value="">Select your experience level</option>
                  <option value="none">None</option>
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="roboticsBackground">Robotics Background</label>
                <select
                  id="roboticsBackground"
                  name="roboticsBackground"
                  value={formData.roboticsBackground}
                  onChange={handleChange}
                  disabled={isLoading}
                >
                  <option value="">Select your background</option>
                  <option value="none">None</option>
                  <option value="student">Student</option>
                  <option value="researcher">Researcher</option>
                  <option value="engineer">Engineer</option>
                  <option value="hobbyist">Hobbyist</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="aiMlExperience">AI/ML Experience</label>
                <select
                  id="aiMlExperience"
                  name="aiMlExperience"
                  value={formData.aiMlExperience}
                  onChange={handleChange}
                  disabled={isLoading}
                >
                  <option value="">Select your experience level</option>
                  <option value="none">None</option>
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>
            </>
          )}

          <button type="submit" disabled={isLoading} className="auth-submit-btn">
            {isLoading ? 'Processing...' : (isLogin ? 'Login' : 'Register')}
          </button>

          <div className="auth-toggle">
            <p>
              {isLogin ? "Don't have an account?" : "Already have an account?"}{' '}
              <button
                type="button"
                onClick={() => setIsLogin(!isLogin)}
                disabled={isLoading}
                className="auth-toggle-btn"
              >
                {isLogin ? 'Register' : 'Login'}
              </button>
            </p>
          </div>
        </form>

        <style jsx>{`
          .auth-modal-overlay {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: rgba(0, 0, 0, 0.5);
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 1000;
          }

          .auth-modal {
            width: 90%;
            max-width: 500px;
            background-color: white;
            border-radius: 8px;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
            overflow: hidden;
          }

          .auth-modal.dark {
            background-color: #1a1a1a;
            color: white;
          }

          .auth-modal-header {
            background-color: #1a5d1a;
            color: white;
            padding: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
          }

          .auth-modal-header.dark {
            background-color: #4caf50;
          }

          .auth-modal-close {
            background: none;
            border: none;
            color: white;
            font-size: 24px;
            cursor: pointer;
            padding: 0;
            width: 30px;
            height: 30px;
            display: flex;
            align-items: center;
            justify-content: center;
          }

          .auth-form {
            padding: 20px;
          }

          .form-group {
            margin-bottom: 15px;
          }

          .form-group label {
            display: block;
            margin-bottom: 5px;
            font-weight: 500;
          }

          .form-group input,
          .form-group select {
            width: 100%;
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 4px;
            font-size: 16px;
          }

          .form-group input.dark,
          .form-group select.dark {
            background-color: #333;
            color: white;
            border-color: #555;
          }

          .auth-submit-btn {
            width: 100%;
            padding: 12px;
            background-color: #1a5d1a;
            color: white;
            border: none;
            border-radius: 4px;
            font-size: 16px;
            cursor: pointer;
          }

          .auth-submit-btn:disabled {
            background-color: #ccc;
            cursor: not-allowed;
          }

          .auth-submit-btn.dark {
            background-color: #4caf50;
          }

          .auth-toggle {
            margin-top: 15px;
            text-align: center;
          }

          .auth-toggle-btn {
            background: none;
            border: none;
            color: #1a5d1a;
            cursor: pointer;
            text-decoration: underline;
          }

          .auth-toggle-btn.dark {
            color: #4caf50;
          }
        `}</style>
      </div>
    </div>
  );
};

export default AuthModal;