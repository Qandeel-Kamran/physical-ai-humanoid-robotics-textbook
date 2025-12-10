import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const ChatbotInjector = () => {
  const location = useLocation();

  useEffect(() => {
    // Dynamically load the chatbot component when the page loads
    const loadChatbot = async () => {
      // Only load on client side
      if (typeof window !== 'undefined') {
        // Check if chatbot is already loaded to avoid duplicates
        if (!document.querySelector('#chatbot-widget')) {
          const ChatbotWidget = (await import('../components/ChatbotWidget')).default;

          // Create a container for the chatbot
          const chatbotContainer = document.createElement('div');
          chatbotContainer.id = 'chatbot-widget';
          document.body.appendChild(chatbotContainer);

          // Render the component
          const React = await import('react');
          const ReactDOM = (await import('react-dom/client')).createRoot;

          const root = ReactDOM(chatbotContainer);
          root.render(React.createElement(ChatbotWidget));
        }
      }
    };

    loadChatbot();
  }, [location.pathname]);

  return null;
};

export default ChatbotInjector;