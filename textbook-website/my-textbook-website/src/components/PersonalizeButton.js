import React, { useState, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

const PersonalizeButton = ({ chapterContent, onPersonalize }) => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [userProfile, setUserProfile] = useState(null);
  const { colorMode } = useColorMode();

  // Check if user is logged in and get their profile
  useEffect(() => {
    const storedProfile = localStorage.getItem('userProfile');
    if (storedProfile) {
      try {
        setUserProfile(JSON.parse(storedProfile));
      } catch (e) {
        console.error('Error parsing user profile:', e);
      }
    }
  }, []);

  const handlePersonalize = async () => {
    if (!userProfile) {
      alert('Please log in to personalize content');
      return;
    }

    setIsLoading(true);

    try {
      // Call the backend to personalize the content
      const response = await fetch('/api/v1/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: chapterContent,
          user_profile: userProfile
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to personalize content');
      }

      const data = await response.json();

      // Update the content with personalized version
      onPersonalize(data.personalized_content);
      setIsPersonalized(true);

      // Store the personalization state
      localStorage.setItem(`personalized_${location.pathname}`, 'true');
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Failed to personalize content. Using original content.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    // Reset to original content
    onPersonalize(chapterContent);
    setIsPersonalized(false);
    localStorage.removeItem(`personalized_${location.pathname}`);
  };

  if (!userProfile) {
    return (
      <div className={`personalize-notice ${colorMode}`}>
        <p>Log in to personalize this content based on your background</p>
        <style jsx>{`
          .personalize-notice {
            padding: 10px;
            margin: 10px 0;
            background-color: #e8f5e8;
            border-left: 4px solid #4caf50;
            border-radius: 4px;
          }
          .personalize-notice.dark {
            background-color: #1a3a1a;
          }
        `}</style>
      </div>
    );
  }

  return (
    <div className={`personalize-controls ${colorMode}`}>
      {!isPersonalized ? (
        <button
          onClick={handlePersonalize}
          disabled={isLoading}
          className={`personalize-btn ${colorMode}`}
        >
          {isLoading ? 'Personalizing...' : 'Personalize Content'}
        </button>
      ) : (
        <button
          onClick={handleReset}
          className={`reset-btn ${colorMode}`}
        >
          Reset to Original
        </button>
      )}

      <style jsx>{`
        .personalize-controls {
          margin: 15px 0;
        }

        .personalize-btn, .reset-btn {
          padding: 8px 16px;
          border: 1px solid #1a5d1a;
          background-color: white;
          color: #1a5d1a;
          border-radius: 4px;
          cursor: pointer;
          font-size: 14px;
        }

        .personalize-btn:hover, .reset-btn:hover {
          background-color: #1a5d1a;
          color: white;
        }

        .personalize-btn:disabled {
          background-color: #ccc;
          cursor: not-allowed;
        }

        .personalize-btn.dark, .reset-btn.dark {
          background-color: #1a1a1a;
          color: #4caf50;
          border-color: #4caf50;
        }

        .personalize-btn.dark:hover, .reset-btn.dark:hover {
          background-color: #4caf50;
          color: #1a1a1a;
        }
      `}</style>
    </div>
  );
};

export default PersonalizeButton;