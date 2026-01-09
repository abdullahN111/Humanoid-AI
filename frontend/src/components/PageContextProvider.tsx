import React, { createContext, useContext, ReactNode, useEffect, useState } from 'react';
import { PageContext } from '../services/api/types';

interface PageContextType {
  pageContext: PageContext;
  updatePageContext: (context: Partial<PageContext>) => void;
}

const PageContextContext = createContext<PageContextType | undefined>(undefined);

interface PageContextProviderProps {
  children: ReactNode;
  initialContext?: Partial<PageContext>;
}

export const PageContextProvider: React.FC<PageContextProviderProps> = ({ children, initialContext }) => {
  const [pageContext, setPageContext] = useState<PageContext>({
    url: initialContext?.url || (typeof window !== 'undefined' ? window.location.href : ''),
    title: initialContext?.title || (typeof window !== 'undefined' ? document.title : ''),
    section: initialContext?.section,
    metadata: initialContext?.metadata,
  });

  useEffect(() => {
    // Update page context when URL or title changes
    const updateContext = () => {
      setPageContext(prev => ({
        ...prev,
        url: window.location.href,
        title: document.title,
      }));
    };

    // Listen for URL changes (SPA navigation)
    window.addEventListener('popstate', updateContext);
    const originalPushState = history.pushState;
    const originalReplaceState = history.replaceState;

    history.pushState = function (...args) {
      // @ts-ignore
      const result = originalPushState.apply(this, args);
      updateContext();
      return result;
    };

    history.replaceState = function (...args) {
      // @ts-ignore
      const result = originalReplaceState.apply(this, args);
      updateContext();
      return result;
    };

    // Initial context setup
    updateContext();

    // Cleanup
    return () => {
      window.removeEventListener('popstate', updateContext);
      history.pushState = originalPushState;
      history.replaceState = originalReplaceState;
    };
  }, []);

  const updatePageContext = (context: Partial<PageContext>) => {
    setPageContext(prev => ({
      ...prev,
      ...context,
    }));
  };

  return (
    <PageContextContext.Provider value={{ pageContext, updatePageContext }}>
      {children}
    </PageContextContext.Provider>
  );
};

export const usePageContext = () => {
  const context = useContext(PageContextContext);
  if (!context) {
    throw new Error('usePageContext must be used within a PageContextProvider');
  }
  return context;
};