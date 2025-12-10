// Chatbot loader script
(function() {
  'use strict';

  // Create the chatbot container
  function initializeChatbot() {
    // Check if the chatbot is already loaded
    if (document.getElementById('chatbot-container')) {
      return;
    }

    // Create the chatbot container
    const chatbotContainer = document.createElement('div');
    chatbotContainer.id = 'chatbot-container';
    document.body.appendChild(chatbotContainer);

    // Load the React app that contains the chatbot
    // For now, we'll use a simple HTML/CSS/JS implementation
    createSimpleChatbot();
  }

  function createSimpleChatbot() {
    // Create the chatbot HTML structure
    const chatbotHTML = `
      <div id="chatbot-widget" style="position: fixed; bottom: 20px; right: 20px; z-index: 1000;">
        <div id="chatbot-toggle" style="width: 60px; height: 60px; border-radius: 50%; border: none; background-color: #1a5d1a; color: white; font-size: 24px; cursor: pointer; box-shadow: 0 4px 8px rgba(0,0,0,0.2); display: flex; align-items: center; justify-content: center;">💬</div>

        <div id="chatbot-window" style="display: none; width: 350px; height: 500px; background-color: white; border-radius: 10px; box-shadow: 0 4px 12px rgba(0,0,0,0.15); position: absolute; bottom: 70px; right: 0; flex-direction: column; overflow: hidden;">
          <div id="chatbot-header" style="background-color: #1a5d1a; color: white; padding: 15px; display: flex; justify-content: space-between; align-items: center;">
            <h3 style="margin: 0; font-size: 16px;">Textbook Assistant</h3>
            <button id="chatbot-close" style="background: none; border: none; color: white; font-size: 20px; cursor: pointer;">×</button>
          </div>
          <div id="chatbot-messages" style="flex: 1; overflow-y: auto; padding: 15px; display: flex; flex-direction: column; gap: 10px; background-color: #fff;">
            <div class="chatbot-welcome" style="text-align: center; color: #666; font-style: italic; margin-top: 50px;">
              <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
              <p>Ask me anything about the content in this textbook.</p>
            </div>
          </div>
          <div id="chatbot-input" style="padding: 15px; border-top: 1px solid #eee; display: flex; gap: 10px; background-color: #fff;">
            <textarea id="chatbot-input-text" placeholder="Ask a question about the textbook..." style="flex: 1; padding: 10px; border: 1px solid #ddd; border-radius: 18px; resize: none; height: 50px;"></textarea>
            <button id="chatbot-send" style="padding: 10px 15px; background-color: #1a5d1a; color: white; border: none; border-radius: 18px; cursor: pointer;">Send</button>
          </div>
        </div>
      </div>
    `;

    // Add the HTML to the container
    document.getElementById('chatbot-container').innerHTML = chatbotHTML;

    // Add event listeners
    const toggleBtn = document.getElementById('chatbot-toggle');
    const chatWindow = document.getElementById('chatbot-window');
    const closeBtn = document.getElementById('chatbot-close');
    const sendBtn = document.getElementById('chatbot-send');
    const inputText = document.getElementById('chatbot-input-text');
    const messagesContainer = document.getElementById('chatbot-messages');

    // Toggle chatbot visibility
    toggleBtn.addEventListener('click', function() {
      chatWindow.style.display = chatWindow.style.display === 'block' ? 'none' : 'block';
    });

    // Close chatbot
    closeBtn.addEventListener('click', function() {
      chatWindow.style.display = 'none';
    });

    // Send message
    sendBtn.addEventListener('click', sendMessage);
    inputText.addEventListener('keypress', function(e) {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        sendMessage();
      }
    });

    function sendMessage() {
      const message = inputText.value.trim();
      if (!message) return;

      // Add user message to chat
      addMessage(message, 'user');
      inputText.value = '';

      // Show loading indicator
      const loadingMsg = addMessage('Thinking...', 'bot', true);

      // Call the API to get response
      fetch('/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: message,
          user_context: null
        }),
      })
      .then(response => response.json())
      .then(data => {
        // Remove loading indicator
        messagesContainer.removeChild(loadingMsg);

        // Add bot response
        addMessage(data.answer, 'bot');
      })
      .catch(error => {
        // Remove loading indicator
        messagesContainer.removeChild(loadingMsg);

        // Add error message
        addMessage('Sorry, I encountered an error. Please try again.', 'bot');
      });
    }

    function addMessage(content, sender, isLoading = false) {
      const messageDiv = document.createElement('div');
      messageDiv.classList.add('message', sender);
      messageDiv.style.maxWidth = '80%';
      messageDiv.style.padding = '10px 15px';
      messageDiv.style.borderRadius = '18px';
      messageDiv.style.marginBottom = '10px';

      if (sender === 'user') {
        messageDiv.style.alignSelf = 'flex-end';
        messageDiv.style.backgroundColor = '#1a5d1a';
        messageDiv.style.color = 'white';
      } else {
        messageDiv.style.alignSelf = 'flex-start';
        messageDiv.style.backgroundColor = '#f0f0f0';
        messageDiv.style.color = '#333';
      }

      const contentPara = document.createElement('p');
      contentPara.style.margin = '0';

      if (isLoading) {
        contentPara.innerHTML = '<em>Thinking...</em>';
      } else {
        contentPara.textContent = content;
      }

      messageDiv.appendChild(contentPara);
      messagesContainer.appendChild(messageDiv);

      // Scroll to bottom
      messagesContainer.scrollTop = messagesContainer.scrollHeight;

      if (isLoading) {
        return messageDiv; // Return the loading element for removal later
      }
    }
  }

  // Initialize the chatbot when the page loads
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeChatbot);
  } else {
    initializeChatbot();
  }
})();