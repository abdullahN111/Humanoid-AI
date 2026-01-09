import React, { useState, useEffect } from 'react';

interface SelectedTextHandlerProps {
  children: React.ReactNode;
}

interface SelectedTextContextType {
  selectedText: string;
  setSelectedText: (text: string) => void;
}

// Create context
const SelectedTextContext = React.createContext<SelectedTextContextType | undefined>(undefined);

export const SelectedTextProvider: React.FC<SelectedTextHandlerProps> = ({ children }) => {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection ? selection.toString().trim() : '';
      setSelectedText(text);
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    // Cleanup event listeners on unmount
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  return (
    <SelectedTextContext.Provider value={{ selectedText, setSelectedText }}>
      {children}
    </SelectedTextContext.Provider>
  );
};

export const useSelectedText = () => {
  const context = React.useContext(SelectedTextContext);
  if (!context) {
    throw new Error('useSelectedText must be used within a SelectedTextProvider');
  }
  return context;
};

// Standalone component for handling selected text (if needed separately)
export const SelectedTextHandler: React.FC = () => {
  const { selectedText, setSelectedText } = useSelectedText();

  return null; // This component doesn't render anything, just handles the logic
};