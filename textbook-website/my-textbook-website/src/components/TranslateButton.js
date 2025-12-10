import React, { useState } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

const TranslateButton = ({ chapterContent, onTranslate }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [targetLanguage, setTargetLanguage] = useState('ur');
  const { colorMode } = useColorMode();

  const handleTranslate = async () => {
    setIsLoading(true);

    try {
      // Call the backend to translate the content
      const response = await fetch('/api/v1/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: chapterContent,
          target_language: targetLanguage
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to translate content');
      }

      const data = await response.json();

      if (data.status === 'success') {
        // Update the content with translated version
        onTranslate(data.translated_content);
        setIsTranslated(true);

        // Store the translation state
        localStorage.setItem(`translated_${location.pathname}`, targetLanguage);
      } else {
        throw new Error(data.message || 'Translation failed');
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert(`Translation failed: ${error.message}. Using original content.`);
      // Don't change content on error
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    // Reset to original content
    onTranslate(chapterContent);
    setIsTranslated(false);
    localStorage.removeItem(`translated_${location.pathname}`);
  };

  const handleLanguageChange = (e) => {
    setTargetLanguage(e.target.value);
  };

  return (
    <div className={`translate-controls ${colorMode}`}>
      <select
        value={targetLanguage}
        onChange={handleLanguageChange}
        disabled={isLoading || isTranslated}
        className={`translate-language-select ${colorMode}`}
      >
        <option value="ur">Urdu</option>
        {/* Add more languages in the future */}
      </select>

      {!isTranslated ? (
        <button
          onClick={handleTranslate}
          disabled={isLoading}
          className={`translate-btn ${colorMode}`}
        >
          {isLoading ? 'Translating...' : 'Translate to Urdu'}
        </button>
      ) : (
        <button
          onClick={handleReset}
          className={`reset-btn ${colorMode}`}
        >
          Show Original
        </button>
      )}

      <style jsx>{`
        .translate-controls {
          margin: 15px 0;
          display: flex;
          gap: 10px;
          align-items: center;
          flex-wrap: wrap;
        }

        .translate-language-select {
          padding: 8px 12px;
          border: 1px solid #ddd;
          border-radius: 4px;
          background-color: white;
          color: black;
        }

        .translate-language-select.dark {
          background-color: #333;
          color: white;
          border-color: #555;
        }

        .translate-btn, .reset-btn {
          padding: 8px 16px;
          border: 1px solid #1a5d1a;
          background-color: white;
          color: #1a5d1a;
          border-radius: 4px;
          cursor: pointer;
          font-size: 14px;
        }

        .translate-btn:hover, .reset-btn:hover {
          background-color: #1a5d1a;
          color: white;
        }

        .translate-btn:disabled {
          background-color: #ccc;
          cursor: not-allowed;
        }

        .translate-btn.dark, .reset-btn.dark {
          background-color: #1a1a1a;
          color: #4caf50;
          border-color: #4caf50;
        }

        .translate-btn.dark:hover, .reset-btn.dark:hover {
          background-color: #4caf50;
          color: #1a1a1a;
        }
      `}</style>
    </div>
  );
};

export default TranslateButton;