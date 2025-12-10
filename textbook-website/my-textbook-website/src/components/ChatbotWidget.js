import React, { useState, useEffect, useRef } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const { colorMode } = useColorMode();

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { type: 'user', content: inputValue, timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get user context if available
      const userContext = getUserContext();

      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          user_context: userContext
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response from chatbot');
      }

      const data = await response.json();
      const botMessage = {
        type: 'bot',
        content: data.answer,
        sources: data.sources,
        timestamp: new Date(),
        confidence: data.confidence
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        type: 'bot',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const getUserContext = () => {
    // In a real implementation, this would get user context from auth
    // For now, return null or dummy data
    return null;
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      {isOpen ? (
        <div className={`chatbot-window ${colorMode}`}>
          <div className="chatbot-header">
            <h3>Textbook Assistant</h3>
            <button className="chatbot-close" onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
                <p>Ask me anything about the content in this textbook.</p>
              </div>
            ) : (
              messages.map((msg, index) => (
                <div key={index} className={`message ${msg.type}`}>
                  <div className="message-content">
                    {msg.type === 'bot' && msg.sources && msg.sources.length > 0 && (
                      <div className="sources">
                        <small>Sources: {msg.sources.map(s => s.metadata?.title || s.metadata?.chapter).join(', ')}</small>
                      </div>
                    )}
                    <p>{msg.content}</p>
                  </div>
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>
          <div className="chatbot-input">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the textbook..."
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={isLoading || !inputValue.trim()}>
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      ) : null}

      <button className={`chatbot-toggle ${colorMode}`} onClick={toggleChat}>
        💬
      </button>

      <style jsx>{`
        .chatbot-container {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 1000;
        }

        .chatbot-toggle {
          width: 60px;
          height: 60px;
          border-radius: 50%;
          border: none;
          background-color: #1a5d1a;
          color: white;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 8px rgba(0,0,0,0.2);
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .chatbot-toggle.dark {
          background-color: #4caf50;
        }

        .chatbot-window {
          width: 350px;
          height: 500px;
          background-color: white;
          border-radius: 10px;
          box-shadow: 0 4px 12px rgba(0,0,0,0.15);
          display: flex;
          flex-direction: column;
          overflow: hidden;
        }

        .chatbot-window.dark {
          background-color: #1a1a1a;
          color: white;
        }

        .chatbot-header {
          background-color: #1a5d1a;
          color: white;
          padding: 15px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .chatbot-header h3 {
          margin: 0;
          font-size: 16px;
        }

        .chatbot-header.dark {
          background-color: #4caf50;
        }

        .chatbot-close {
          background: none;
          border: none;
          color: white;
          font-size: 20px;
          cursor: pointer;
        }

        .chatbot-messages {
          flex: 1;
          overflow-y: auto;
          padding: 15px;
          display: flex;
          flex-direction: column;
          gap: 10px;
        }

        .chatbot-messages.dark {
          background-color: #1a1a1a;
        }

        .chatbot-welcome {
          text-align: center;
          color: #666;
          font-style: italic;
          margin-top: 50px;
        }

        .chatbot-welcome.dark {
          color: #aaa;
        }

        .message {
          max-width: 80%;
          padding: 10px 15px;
          border-radius: 18px;
          margin-bottom: 10px;
        }

        .message.user {
          align-self: flex-end;
          background-color: #1a5d1a;
          color: white;
        }

        .message.user.dark {
          background-color: #4caf50;
        }

        .message.bot {
          align-self: flex-start;
          background-color: #f0f0f0;
          color: #333;
        }

        .message.bot.dark {
          background-color: #333;
          color: white;
        }

        .sources {
          font-size: 12px;
          color: #666;
          margin-bottom: 5px;
        }

        .sources.dark {
          color: #aaa;
        }

        .chatbot-input {
          padding: 15px;
          border-top: 1px solid #eee;
          display: flex;
          gap: 10px;
        }

        .chatbot-input.dark {
          border-top: 1px solid #444;
          background-color: #1a1a1a;
        }

        .chatbot-input textarea {
          flex: 1;
          padding: 10px;
          border: 1px solid #ddd;
          border-radius: 18px;
          resize: none;
          height: 50px;
        }

        .chatbot-input textarea.dark {
          background-color: #333;
          color: white;
          border: 1px solid #555;
        }

        .chatbot-input button {
          padding: 10px 15px;
          background-color: #1a5d1a;
          color: white;
          border: none;
          border-radius: 18px;
          cursor: pointer;
        }

        .chatbot-input button:disabled {
          background-color: #ccc;
          cursor: not-allowed;
        }

        .chatbot-input button.dark {
          background-color: #4caf50;
        }
      `}</style>
    </div>
  );
};

export default ChatbotWidget;